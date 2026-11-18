"""Put a 7-ft pool table, a cue and three balls at the stock Inspect Robots YAM station."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PHYSICS_DT
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as fixture
from . import layout as L
from .grading import PoolGrade

CURRENT = None                                      # the live embodiment, for the scripted demonstration
FULL_WIDTH = 2 * abs(FINGER_OPEN)                   # gripper reading 1.0 = pads 93.9 mm apart
TIP_REACH = L.BALL_R + .006                         # a cue-ball contact within this of the tip's apex is the tip's
TRACE_SPAN = 48                                     # physics steps logged either side of the strike
# Control runs (README): DEMO_VARIANT=one_hand|no_slide|pinch_bridge changes the scripted demonstration.
DEMO_VARIANT = os.environ.get("POOL_DEMO_VARIANT", "")


def set_bounce_cfg(env_cfg, value):
    """Set ``bounce_threshold_velocity`` on whichever part of the simulation config has it (Isaac Lab versions
    differ: ``sim.physx`` in older ones). Returns what was done; never raises."""
    done = []
    try:
        sim = getattr(env_cfg, "sim", None)
        candidates = [("sim", sim)]
        for name in dir(sim) if sim is not None else ():
            if name.startswith("_"):
                continue
            try:
                candidates.append((f"sim.{name}", getattr(sim, name)))
            except Exception:
                pass
        for path, obj in candidates:
            if obj is not None and not isinstance(obj, (int, float, str, bool)) and hasattr(obj, "bounce_threshold_velocity"):
                setattr(obj, "bounce_threshold_velocity", float(value))
                done.append(path)
    except Exception as error:
        return {"set": done, "error": repr(error)}
    return {"set": done}


def set_bounce_usd(value):
    """Set ``physxScene:bounceThreshold`` on every PhysicsScene prim of the stage and read it back. Never raises."""
    out = {}
    try:
        import omni.usd
        from pxr import Sdf

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.GetTypeName() != "PhysicsScene":
                continue
            path = str(prim.GetPath())
            try:
                try:
                    from pxr import PhysxSchema

                    PhysxSchema.PhysxSceneAPI.Apply(prim).CreateBounceThresholdAttr().Set(float(value))
                except Exception:
                    prim.CreateAttribute("physxScene:bounceThreshold", Sdf.ValueTypeNames.Float).Set(float(value))
                out[path] = prim.GetAttribute("physxScene:bounceThreshold").Get()
            except Exception as error:
                out[path] = repr(error)
        if not out:
            out["error"] = "no PhysicsScene prim found"
    except Exception as error:
        out["error"] = repr(error)
    return out


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class PoolArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_pool_bridge",
                            environment_id="isaaclab-arena-0.3.1-yam-pool-bridge",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station at the long rail of a 7-ft pool table, with a cue "
                                 "and three balls. Observations are overhead RGB, both wrist RGB views, and robot "
                                 "joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.live = False
        self.bounce_applied = {}                    # where the PhysX bounce threshold was set (RESULT.json)
        CURRENT = self

    # -- building the simulation ---------------------------------------------------------------------------------
    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        from .recording import review_camera

        def make_env(embodiment, *, device, control_hz, **kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene

            # The station's own table is replaced by the robot's bench in front of the pool table (in the USD).
            objects = [o for o in scenes.task_room("station", embodiment.rig.top_cam) if o.name != "table"]
            objects.append(Object(name="pool_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            stock_cfg = scenes.env_cfg_callback(control_hz)

            def configure(env_cfg):
                env_cfg = stock_cfg(env_cfg)
                # PhysX makes impacts slower than its bounce threshold inelastic; 0.5 m/s (the usual default)
                # would deaden slow cushion rebounds and soft ball contacts. Set it wherever this Isaac Lab keeps
                # it; never fail the trial over it (the USD scene attribute is also set after the build).
                self.bounce_applied["cfg"] = set_bounce_cfg(env_cfg, fixture.BOUNCE_THRESHOLD)
                return env_cfg

            env = IsaacLabArenaEnvironment(name="yam_pool_bridge", scene=Scene(objects), embodiment=embodiment,
                                           env_cfg_callback=configure)
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=device)).make_registered()

        with review_camera():
            from yam_arena import scene as scenes

            stock = scenes.make_env
            scenes.make_env = make_env
            try:
                super()._ensure_sim(None)
            finally:
                scenes.make_env = stock
        # Rolling resistance, spin friction and the contact sensors act at every physics substep: Isaac Lab calls
        # write_data_to_sim() once before each of the 12 PhysX steps inside one 20 Hz control period.
        self.bounce_applied["usd"] = set_bounce_usd(fixture.BOUNCE_THRESHOLD)
        scene = self._base.scene
        stock_write = scene.write_data_to_sim

        def write_data_to_sim(*args, **kwargs):
            stock_write(*args, **kwargs)
            if self.live:
                self.physics_step()

        scene.write_data_to_sim = write_data_to_sim

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        found = [str(p.GetPath()) for p in stage.Traverse()
                 if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(found) != 1:
            raise RuntimeError(f"Expected one pool fixture, found {found}")
        actors = found[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.ACTORS}
        self.ball_rows = [self.rows[fixture.BALL_BODY[n]] for n in L.BALLS]
        self.all_ids = self._torch.arange(len(paths), dtype=self._torch.int32, device=self._base.device)
        links = asset_link_paths()
        self.link_names = list(links)
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        cue = f"{actors}/cue_stick"
        ball = {n: f"{actors}/{fixture.BALL_BODY[n]}" for n in L.BALLS}
        view = self.physics.create_rigid_contact_view
        # one view per contact pair (a shared view once mislabelled a force for us)
        self.contact = {
            "interarm": view(arm_links["left"], [arm_links["right"]] * len(arm_links["left"])),
            **{f"cue_{n}": view([cue], [[ball[n]]]) for n in L.BALLS},
            **{f"white_{n}": view([ball["cue"]], [[ball[n]]]) for n in ("one", "three")},
            **{f"arms_{n}": view(arm_links["left"] + arm_links["right"], [[ball[n]]] * (2 * len(links))) for n in L.BALLS},
            **{f"{arm}_cue": view(arm_links[arm], [[cue]] * len(links)) for arm in fixture.ARMS},
        }

    def _read(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(poses).all() and np.isfinite(velocities).all()):
            raise RuntimeError("Nonfinite cue or ball state")
        self.poses, self.velocities = poses, velocities

    def cue_pose(self):
        """(tip apex, unit axis toward the tip, linear velocity of the cue's body origin)."""
        row = self.rows["cue_stick"]
        rotation = quat_xyzw_to_matrix(self.poses[row, 3:7])
        return self.poses[row, :3].copy(), rotation[:, 0].copy(), self.velocities[row, :3].copy()

    def balls(self):
        return {n: (self.poses[r, :3].copy(), self.velocities[r, :3].copy()) for n, r in zip(L.BALLS, self.ball_rows)}

    # -- every physics step --------------------------------------------------------------------------------------
    def _force(self, key):
        return np.linalg.norm(array(self.contact[key].get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)

    def physics_step(self):
        """Read the contacts of the last physics step into this control step's events, then apply the cloth's
        rolling resistance and spin friction to the balls for the next one."""
        self.substep += 1
        self._read()
        tip, axis, _ = self.cue_pose()
        ev = self.events
        white_force = float(self._force("cue_cue").sum())
        for n in L.BALLS:
            f = white_force if n == "cue" else float(self._force(f"cue_{n}").sum())
            if f <= 0.:
                continue
            centre = self.poses[self.ball_rows[L.BALLS.index(n)], :3]
            at_tip = np.linalg.norm(centre - tip) <= TIP_REACH and (centre - tip) @ axis > 0.
            if n == "cue" and at_tip:
                ev["tip_cue_ball"] += 1
                ev["impulse_ns"] += f * PHYSICS_DT
                ev["peak_tip_force_n"] = max(ev["peak_tip_force_n"], f)
            elif at_tip:
                ev["tip_object_balls"] = tuple(sorted(set(ev["tip_object_balls"]) | {n}))
            else:
                ev["cue_side_balls"] = tuple(sorted(set(ev["cue_side_balls"]) | {n}))
        for n in ("one", "three"):
            if float(self._force(f"white_{n}").sum()) > 0. and n not in ev["cue_ball_hits"]:
                ev["cue_ball_hits"] = ev["cue_ball_hits"] + (n,)
        if float(self._force("left_cue").sum()) > 0.:
            ev["left_on_cue"] += 1
        for n in L.BALLS:
            if float(self._force(f"arms_{n}").sum()) > 0.:
                ev["robot_balls"] = tuple(sorted(set(ev["robot_balls"]) | {n}))
        # the last few hundred physics steps around the strike: tip force and the white's speed, every step
        row = self.ball_rows[0]
        self.substep_trace.append([self.substep, round(white_force, 2),
                                   round(float(np.linalg.norm(self.velocities[row, :3])), 4),
                                   round(float(np.linalg.norm(self.velocities[self.rows["cue_stick"], :3])), 4)])
        if self.strike_substep is None and ev["tip_cue_ball"]:
            self.strike_substep = self.substep
        if self.strike_substep is None:
            self.substep_trace = self.substep_trace[-TRACE_SPAN:]
        self._roll()

    def _roll(self):
        """Cloth rolling resistance (0.01, as a force at the centre sized so a rolling ball slows by 0.098 m/s^2)
        and spin friction about the vertical (10 rad/s^2), never reversing a motion within one step."""
        forces = np.zeros((len(self.poses), 3), dtype=np.float32)
        torques = np.zeros_like(forces)
        inertia = .4 * L.BALL_MASS * L.BALL_R ** 2
        for row in self.ball_rows:
            p, v, w = self.poses[row, :3], self.velocities[row, :3], self.velocities[row, 3:]
            if not L.on_cloth(p):
                continue
            speed = float(np.linalg.norm(v[:2]))
            if speed > 1e-4:
                size = min(1.4 * L.BALL_MASS * fixture.ROLL_DECEL, 1.4 * L.BALL_MASS * speed / PHYSICS_DT)
                forces[row, :2] = -size * v[:2] / speed
            if abs(w[2]) > 1e-3:
                torques[row, 2] = -np.sign(w[2]) * min(inertia * fixture.SPIN_DECEL, inertia * abs(w[2]) / PHYSICS_DT)
        torch = self._torch
        device = self._base.device
        self.body_view.apply_forces_and_torques_at_position(
            torch.as_tensor(forces, device=device), torch.as_tensor(torques, device=device), None, self.all_ids, True)

    def _new_events(self):
        from .grading import EMPTY_EVENTS

        self.events = {k: (v if not isinstance(v, tuple) else ()) for k, v in EMPTY_EVENTS.items()}

    # -- trial ---------------------------------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self.live = False
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        top = np.array((0., 0., fixture.TOP))
        poses = self.body_view.get_transforms().clone()
        starts = {"cue_stick": (L.CUE_REST_TIP, ik.matrix_to_quat_xyzw(L.cue_rotation(L.CUE_REST_AXIS)))}
        for n in L.BALLS:
            starts[fixture.BALL_BODY[n]] = (L.BALL_START[n] + (0., 0., .0002), np.array((0., 0., 0., 1.)))
        for name, (pos, quat) in starts.items():
            poses[self.rows[name]] = torch.tensor(np.r_[pos + top, quat], device=poses.device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.all_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.all_ids)
        self.substep = 0
        self.substep_trace = []
        self.strike_substep = None
        self._new_events()
        self.live = True
        observation = super().reset(scene, seed=seed)
        self._read()
        for n, (p, _) in self.balls().items():
            if np.linalg.norm(p - L.BALL_START[n]) > .003:
                raise RuntimeError(f"The {n} ball is off its spot after reset: {p}")
        tip, axis, _ = self.cue_pose()
        if np.linalg.norm(tip - L.CUE_REST_TIP) > .006 or axis @ L.CUE_REST_AXIS < .999:
            raise RuntimeError(f"The cue is not lying on the rail after reset: {tip}, {axis}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.masses = {name: round(float(array(self.body_view.get_masses()).reshape(-1)[row]), 4)
                       for name, row in self.rows.items()}
        self.grading = PoolGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak_hand_on_cue_n = {arm: 0. for arm in fixture.ARMS}
        self.final_grade = None
        self.diagnostics = []
        self.strike_rows = []
        self.substep_trace = []
        self.strike_substep = None
        self._new_events()
        self.joint_pos = np.asarray(observation.state["joint_pos"], dtype=float).copy()
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def hands(self, joints):
        return {arm: ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm]) for arm, i in zip(fixture.ARMS, (0, 7))}

    def sample(self, joints):
        tip, axis, velocity = self.cue_pose()
        return {"balls": self.balls(), "cue_tip": tip, "cue_axis": axis, "cue_velocity": velocity,
                "tcp": self.hands(joints),
                "opening": {arm: float(joints[i]) * FULL_WIDTH for arm, i in zip(fixture.ARMS, (6, 13))}}

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        tip, axis, velocity = self.cue_pose()
        self.demo_state = {"cue_tip": tip, "cue_axis": axis, "cue_velocity": velocity,
                           "balls": {n: p for n, (p, _) in self.balls().items()},
                           "struck": bool(self.final_grade and self.final_grade["struck"])}

    def step(self, action):
        self._new_events()
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.joint_pos = joints.copy()
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(self._force("interarm"))))
        on_cue = {arm: float(self._force(f"{arm}_cue").sum()) for arm in fixture.ARMS}
        for arm in fixture.ARMS:
            self.peak_hand_on_cue_n[arm] = max(self.peak_hand_on_cue_n[arm], on_cue[arm])
        self.tick += 1
        sample = self.sample(joints)
        events = dict(self.events)
        grade = self.grading.update(sample, events)
        self.final_grade = grade
        self._publish_demo_state()
        if self.tick % 5 == 0 or events["tip_cue_ball"] or (grade["struck"] and self.tick - grade["strike_tick"] < 40):
            row = self._diagnose(sample, grade, events, on_cue)
            (self.strike_rows if grade["struck"] and self.tick - grade["strike_tick"] < 40 else self.diagnostics).append(row)
        over = bool(grade["finished"])
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = ("success" if grade["score"] == 1. else "shot over") if over else None
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"pool_grade": grade})

    def _diagnose(self, sample, grade, events, on_cue):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        from .grading import bridge_check, grip_check

        mm = lambda v: [round(1000 * float(x), 1) for x in v]  # noqa: E731
        return {
            "tick": self.tick, "score": grade["score"], "struck": grade["struck"], "legal": grade["legal"],
            "fouls": grade["fouls"], "first_hit": grade["first_hit"], "potted": grade["potted"],
            "balls_mm": {n: mm(p) for n, (p, _) in sample["balls"].items()},
            "ball_speed_m_s": {n: round(float(np.linalg.norm(v)), 4) for n, (_, v) in sample["balls"].items()},
            "cue_tip_mm": mm(sample["cue_tip"]), "cue_axis": [round(float(v), 4) for v in sample["cue_axis"]],
            "cue_speed_m_s": round(float(np.linalg.norm(sample["cue_velocity"])), 4),
            "tcp_mm": {arm: mm(sample["tcp"][arm][0]) for arm in fixture.ARMS},
            "opening_mm": {arm: round(1000 * sample["opening"][arm], 1) for arm in fixture.ARMS},
            "bridge": bridge_check(sample), "grip": grip_check(sample),
            "hand_on_cue_n": {arm: round(on_cue[arm], 2) for arm in fixture.ARMS},
            "events": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in events.items()},
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class PoolScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import PoolReference

        self.reference = PoolReference(variant=DEMO_VARIANT)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the cue's and the
        # balls' poses), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return PoolArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = PoolScripted()
    policy.info = replace(YamScriptedPolicy().info, name="pool_bridge_scripted")
    return policy
