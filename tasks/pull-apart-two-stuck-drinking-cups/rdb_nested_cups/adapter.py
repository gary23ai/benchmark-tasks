"""Add the two stuck cups to the stock Inspect Robots YAM station and grade each step."""

from dataclasses import replace
from pathlib import Path
from weakref import WeakValueDictionary

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import cup_model as cup
from . import fixed_scene as fixture
from . import grading

_RIGS = WeakValueDictionary()  # the scripted demonstration looks its embodiment up here to read cup poses

if (grading.CONTROL_HZ, grading.HOLD_S) != (fixture.CONTROL_HZ, fixture.SUCCESS_HOLD_S):
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate or hold time")


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class CupArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_nested_cups",
                            environment_id="isaaclab-arena-0.3.1-yam-nested-cups",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with two stuck drinking cups on the tabletop. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        _RIGS[self.info.environment_id] = self
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None

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

            objects = scenes.task_room("station", embodiment.rig.top_cam)
            objects.append(Object(name="cup_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_nested_cups", scene=Scene(objects),
                                           embodiment=embodiment, env_cfg_callback=scenes.env_cfg_callback(control_hz))
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

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        actor_roots = [str(p.GetPath()) for p in omni.usd.get_context().get_stage().Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected one set of fixture actors, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.cup_view = self.physics.create_rigid_body_view(actors + "/*")
        self.rows = [list(self.cup_view.prim_paths).index(f"{actors}/{name}") for name in fixture.CUPS]
        links = asset_link_paths()
        self.link_names = list(links)
        paths = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                 for arm in fixture.ARMS}
        targets = [f"{actors}/{name}" for name in fixture.CUPS]
        self.cup_contacts = self.physics.create_rigid_contact_view(
            paths["left"] + paths["right"], [targets] * (len(paths["left"]) + len(paths["right"])))
        self.interarm = self.physics.create_rigid_contact_view(paths["left"], [paths["right"]] * len(paths["left"]))
        # Diagnostic only: the inner cup's contact with the outer cup, to read the fit's wall force directly.
        try:
            self.cup_on_cup = self.physics.create_rigid_contact_view([targets[1]], [[targets[0]]])
        except Exception as error:  # the trial must not depend on a diagnostic
            self.cup_on_cup, self.cup_on_cup_error = None, repr(error)
        # Contact forces are impulses of the last physics step divided by the step, not by the control period.
        self.physics_dt = float(getattr(self._base, "physics_dt", 0.) or 1. / 240.)
        self.actor_names = [path.rsplit("/", 1)[-1] for path in self.cup_view.prim_paths]
        self.probe_contacts = None
        if cup.PROBE:
            try:
                extra = [f"{actors}/{name}" for name in cup.PROBE_BODIES]
                sensors = paths["left"] + paths["right"]
                self.probe_contacts = self.physics.create_rigid_contact_view(sensors, [extra] * len(sensors))
            except Exception as error:
                self.probe_error = repr(error)

    def _read_cups(self):
        self.cups = array(self.cup_view.get_transforms())[self.rows]
        self.cups[:, 2] -= fixture.TOP
        self.velocity = array(self.cup_view.get_velocities())[self.rows]

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        poses = self.cup_view.get_transforms().clone()
        for row, position in zip(self.rows, fixture.RESET_POS, strict=True):
            poses[row] = torch.tensor((*position, 0., 0., 0., 1.), device=poses.device, dtype=poses.dtype)
        indices = torch.arange(len(self.rows), dtype=torch.int32, device=poses.device)
        self.cup_view.set_transforms(poses, indices)
        self.cup_view.set_velocities(torch.zeros_like(self.cup_view.get_velocities()), indices)
        observation = super().reset(scene, seed=seed)
        self._read_cups()
        if not np.allclose(self.cups[:, :3] + (0, 0, fixture.TOP), fixture.RESET_POS, rtol=0, atol=.002):
            raise RuntimeError(f"Fixed cup layout failed to reset: {self.cups[:, :3]}")
        masses = array(self.cup_view.get_masses())[self.rows].reshape(-1)
        if not np.allclose(masses, fixture.RESET_MASS, rtol=0, atol=.0001):
            raise RuntimeError(f"Fixed cup masses failed to reset: {masses}")
        self.grading = grading.StuckCups(table_bounds=fixture.TABLE_BOUNDS)
        self.tick = 0
        # Diagnostics only; neither enters the score.
        self.peak_interarm_n = 0.
        self.last_grade = None
        self.history = []  # one diagnostic row per control step; never shown to a policy
        # Height of the inner cup's base above the outer cup's base after the 0.5 s settle. The authored value
        # is cup_model.SEATED_PITCH and the bond should keep it there.
        self.reset_pitch_mm = float(1000 * (self.cups[1, 2] - self.cups[0, 2]))
        self.reset_height = float((self.cups[1, :3] - self.cups[0, :3]) @ grading.axis_up(self.cups[0, 3:]))
        self.fit = self.reset_fit = self._read_fit()
        self.fit_joint = self._fit_joint()
        # When the bond let go, and whether it was still holding 5 s in and when a robot first touched a cup.
        self.fit_events = {"bond_broke_at_tick": None, "bonded_at_5_s": None, "bonded_at_first_touch": None,
                           "first_touch_tick": None}
        self.reset_materials = self._bound_materials()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def step(self, action):
        commanded = np.asarray(action.data, dtype=float).reshape(-1)
        observation = super().step(action).observation
        self._read_cups()
        joints = observation.state["joint_pos"]
        dt = self.physics_dt
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=dt)), axis=-1))))
        tcp = np.array([ik.tcp_pose(joints[i:i+6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        # Rows are robot links, columns are the two cups.
        forces = array(self.cup_contacts.get_contact_force_matrix(dt=dt)).reshape(-1, len(fixture.CUPS), 3)
        touching = (np.abs(forces) > 1.e-6).any(axis=(0, 2))
        grade = self.grading.update(self.cups, self.velocity, tcp, touching)
        self.fit = self._read_fit()
        events, tick = self.fit_events, self.tick + 1
        if not self.fit["bonded"] and events["bond_broke_at_tick"] is None:
            events["bond_broke_at_tick"] = tick
        if tick == 5 * fixture.CONTROL_HZ:
            events["bonded_at_5_s"] = self.fit["bonded"]
        if touching.any() and events["first_touch_tick"] is None:
            events["first_touch_tick"], events["bonded_at_first_touch"] = tick, self.fit["bonded"]
        self.last_grade = grade
        diagnostics = self._diagnostics(commanded, joints, tcp, forces)
        self.history.append(diagnostics)
        self.tick += 1
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, {**grade, "diag": diagnostics},
                               terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"cup_grade": grade})

    def _finger_row(self):
        """Diagnostic: each finger joint's position, target and effort (the observation only has their mean)."""
        out = {}
        for arm in fixture.ARMS:
            for attribute in ("joint_pos", "joint_pos_target", "applied_torque"):
                try:
                    robot = self._base.scene[f"{arm}_robot"]
                    value = getattr(robot.data, attribute)
                    value = (value.torch if hasattr(value, "torch") else value).detach().cpu().numpy()
                    out[f"{arm}_{attribute}"] = [round(float(v), 5) for v in value[0][self._ids[f"{arm}_robot"][1]]]
                except Exception as error:
                    out[f"{arm}_{attribute}"] = f"error: {error!r}"
        return out

    def _probe_row(self, forces):
        """Extra simulator record for the diagnostic probe. Every reader is guarded: a missing attribute is
        logged as an error string and never stops the run."""
        out, n = {}, len(self.link_names)

        def guarded(key, read):
            try:
                out[key] = read()
            except Exception as error:
                out[key] = f"error: {error!r}"

        def tensor(value):
            value = value.torch if hasattr(value, "torch") else value
            return value.detach().cpu().numpy()

        def bodies():
            poses = array(self.cup_view.get_transforms())
            return {name: [round(float(v), 5) for v in (*pose[:2], pose[2] - fixture.TOP, *pose[3:])]
                    for name, pose in zip(self.actor_names, poses, strict=True)}

        guarded("bodies_xyz_quat_xyzw", bodies)
        for a, arm in enumerate(fixture.ARMS):
            robot_key = f"{arm}_robot"

            def fingers(attribute, robot_key=robot_key):
                robot = self._base.scene[robot_key]
                return [round(float(v), 5) for v in tensor(getattr(robot.data, attribute))[0][self._ids[robot_key][1]]]

            for attribute in ("joint_pos", "joint_vel", "joint_pos_target", "applied_torque", "computed_torque",
                              "joint_effort_target"):
                guarded(f"{arm}_finger_{attribute}", lambda attribute=attribute: fingers(attribute))

            def links(robot_key=robot_key):
                robot = self._base.scene[robot_key]
                poses = tensor(robot.data.body_link_pose_w)[0]
                return {name: [round(float(v), 5) for v in (*pose[:2], pose[2] - fixture.TOP, *pose[3:])]
                        for name, pose in zip(robot.body_names, poses, strict=True)
                        if name in ("gripper", "tip_left", "tip_right")}

            guarded(f"{arm}_link_xyz_quat_xyzw", links)

            part = forces[a * n:(a + 1) * n]
            out[f"{arm}_force_on_link_from_cup_n"] = {
                f"{link}<-{name}": [round(float(v), 2) for v in part[i, c]]
                for i, link in enumerate(self.link_names) for c, name in enumerate(fixture.CUPS)
                if float(np.linalg.norm(part[i, c])) > .01}

        def probe_bodies():
            if self.probe_contacts is None:
                return getattr(self, "probe_error", "no view")
            names = list(cup.PROBE_BODIES)
            matrix = array(self.probe_contacts.get_contact_force_matrix(dt=self.physics_dt)).reshape(-1, len(names), 3)
            found = {}
            for a, arm in enumerate(fixture.ARMS):
                for i, link in enumerate(self.link_names):
                    for c, name in enumerate(names):
                        force = matrix[a * n + i, c]
                        if float(np.linalg.norm(force)) > .01:
                            found[f"{arm}.{link}<-{name}"] = [round(float(v), 2) for v in force]
            return found

        guarded("force_on_link_from_probe_body_n", probe_bodies)
        return out

    def _bound_materials(self):
        """Diagnostic: the physics material the simulator's stage actually binds to one wall panel and one base
        disc of each cup, with its friction and compliant-contact values."""
        try:
            import omni.usd
            from pxr import UsdShade

            stage, out = omni.usd.get_context().get_stage(), {}
            for prim in stage.Traverse():
                name = prim.GetName()
                if name not in ("outer_wall0", "inner_wall0", "outer_base", "inner_base") or prim.GetTypeName() == "Material":
                    continue
                material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")[0]
                values = {a.GetName(): a.Get() for a in material.GetPrim().GetAttributes()
                          if a.GetName().startswith(("physics:", "physxMaterial:"))} if material else {}
                out[name] = {"material": str(material.GetPath()) if material else None,
                             "visibility": str(prim.GetAttribute("visibility").Get()),
                             **{k: (float(v) if isinstance(v, (int, float)) else str(v)) for k, v in values.items()}}
            return out
        except Exception as error:
            return {"error": repr(error)}

    def _read_fit(self):
        """Diagnostic: is the bond between the cups still holding? While it holds the inner cup cannot move in
        the outer one, so "bonded" is read from their relative pose: the inner cup's height in the outer cup
        within 0.2 mm of its value at reset, and no more than 0.2 mm off the outer cup's axis. The cup-on-cup
        contact force (normal forces only) is logged alongside; it should be zero while the bond holds."""
        up = grading.axis_up(self.cups[0, 3:])
        offset = self.cups[1, :3] - self.cups[0, :3]
        height = float(offset @ up)
        off_axis = float(np.linalg.norm(offset - height * up))
        start = getattr(self, "reset_height", height)
        out = {"inner_above_outer_mm": round(1000 * height, 3), "off_axis_mm": round(1000 * off_axis, 3),
               "bonded": bool(abs(height - start) < .0002 and off_axis < .0002)}
        if self.cup_on_cup is not None:
            try:
                force = array(self.cup_on_cup.get_contact_force_matrix(dt=self.physics_dt)).reshape(-1, 3).sum(axis=0)
                out["cup_on_cup_normal_force_n"] = force.round(3).tolist()
            except Exception as error:
                self.cup_on_cup, out["cup_on_cup_error"] = None, repr(error)
        return out

    def _fit_joint(self):
        """Diagnostic: the bond as the simulator's stage holds it (path, break force and moment, enabled)."""
        try:
            import omni.usd

            for prim in omni.usd.get_context().get_stage().Traverse():
                if prim.GetName() == "fit" and prim.GetTypeName() == "PhysicsFixedJoint":
                    read = lambda name: prim.GetAttribute(name).Get()  # noqa: E731
                    return {"path": str(prim.GetPath()), "break_force_n": float(read("physics:breakForce")),
                            "break_torque_nm": float(read("physics:breakTorque")),
                            "enabled": bool(read("physics:jointEnabled")),
                            "bodies": [str(t) for rel in ("physics:body0", "physics:body1")
                                       for t in prim.GetRelationship(rel).GetTargets()]}
            return {"error": "no PhysicsFixedJoint named fit on the stage"}
        except Exception as error:
            return {"error": repr(error)}

    def _diagnostics(self, commanded, joints, tcp, forces):
        """Simulator-side record of one step for the result files. Diagnostics only: nothing here reaches a
        policy or the score."""
        n = len(self.link_names)
        row = {"tick": self.tick + 1, "cups": {}, "arms": {}}
        ups = [grading.axis_up(pose[3:]) for pose in self.cups]
        for i, name in enumerate(fixture.CUPS):
            row["cups"][name] = {"base_mm": (1000 * self.cups[i, :3]).round(2).tolist(),
                                 "tilt_deg": round(float(np.degrees(np.arccos(np.clip(ups[i][2], -1., 1.)))), 2),
                                 "speed_mm_s": round(1000 * float(np.linalg.norm(self.velocity[i, :3])), 1)}
        offset = self.cups[1, :3] - self.cups[0, :3]
        row["inner_above_outer_mm"] = round(1000 * float(offset @ ups[0]), 3)
        row["fit"] = self.fit
        if cup.PROBE:
            row["probe"] = self._probe_row(forces)
        row["fingers"] = self._finger_row()
        row["inner_off_axis_mm"] = round(1000 * float(np.linalg.norm(offset - (offset @ ups[0]) * ups[0])), 3)
        for a, arm in enumerate(fixture.ARMS):
            target = ik.tcp_pose(commanded[7 * a:7 * a + 6], fixture.BASES[arm])[0]
            part = forces[a * n:(a + 1) * n]  # this arm's links x cups x 3
            contact = {}
            for c, name in enumerate(fixture.CUPS):
                per_link = np.linalg.norm(part[:, c], axis=-1)
                contact[name] = {"net_n": part[:, c].sum(axis=0).round(2).tolist(),
                                 "links_n": {link: round(float(f), 2)
                                             for link, f in zip(self.link_names, per_link, strict=True) if f > .01}}
            row["arms"][arm] = {"tcp_measured_mm": (1000 * tcp[a]).round(2).tolist(),
                                "tcp_commanded_mm": (1000 * target).round(2).tolist(),
                                "opening_measured": round(float(joints[7 * a + 6]), 4),
                                "opening_commanded": round(float(commanded[7 * a + 6]), 4),
                                "cup_contact": contact}
        return row

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class CupScripted(PolicyBase):
    """Closed-loop demonstration. It reads the cups' poses from the embodiment and the joint and gripper
    readings from the observation, and acts only through the 14 joint commands."""

    def bind(self, embodiment_info):
        self.env = _RIGS[embodiment_info.environment_id]

    def reset(self, scene):
        if cup.PROBE:
            from .probe import ProbeReference as CupReference
        else:
            from .scripted import CupReference
        self.reference = CupReference(self.env)
        self.actions = self.reference.sequence()

    def act(self, observation):
        self.reference.measured = np.asarray(observation.state["joint_pos"], dtype=float)
        return next(self.actions)


def make_embodiment(**kwargs):
    return CupArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy
    policy = CupScripted()
    policy.info = replace(YamScriptedPolicy().info, name="nested_cups_scripted")
    return policy
