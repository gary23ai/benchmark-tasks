"""Add the Dounce grinder, its cold block and the lab jack to the stock Inspect Robots YAM station.

Each control step, before the physics runs, the liquid law's damping (the D6 joint's drive between tube and
pestle) is set from how much of the pestle's head is under the computed surface; PhysX applies it implicitly at
every 240 Hz substep, equal and opposite on the two glass parts. Nothing else is written into the simulation
except the drawn liquid's height (display only).
"""

from dataclasses import replace
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as F
from . import grading
from . import layout as L

if grading.CONTROL_HZ != F.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

CURRENT = None                                       # the live embodiment, for the scripted demonstration
PAD_SPAN_MM = 2000 * abs(FINGER_OPEN)                # pads' separation at a gripper reading of 1.0 (93.9 mm)
SINK_WINDOW = (10, 60)                               # ticks over which the free pestle's sinking speed is measured


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class DounceArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=F.CONTROL_HZ, cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_dounce", environment_id="isaaclab-arena-0.3.1-yam-dounce",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a 7 mL glass Dounce tissue grinder (tight "
                                 "pestle standing in the tube) in an aluminium cold block on a lab jack. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        CURRENT = self

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
            objects.append(Object(name="dounce_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            env = IsaacLabArenaEnvironment(name="yam_dounce", scene=Scene(objects), embodiment=embodiment,
                                           env_cfg_callback=scenes.env_cfg_callback(control_hz))
            return ArenaEnvBuilder(env, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()

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
        from pxr import UsdGeom, UsdPhysics
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()

        def find(name):
            found = [str(p.GetPath()) for p in stage.Traverse()
                     if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
            if len(found) != 1:
                raise RuntimeError(f"Expected one {name}, found {found}")
            return found[0]

        actors = find("Actors")
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in F.ACTORS}
        joint = stage.GetPrimAtPath(find(F.FLUID_JOINT))
        drive = UsdPhysics.DriveAPI(joint, "transX")
        self.damping_attr, self.maxforce_attr = drive.GetDampingAttr(), drive.GetMaxForceAttr()
        self.fluid_info = {"path": str(joint.GetPath()), "type": joint.GetTypeName(),
                           "damping_n_s_m": float(self.damping_attr.Get()), "max_force_n": float(self.maxforce_attr.Get()),
                           "collision_enabled": bool(UsdPhysics.Joint(joint).GetCollisionEnabledAttr().Get())}
        self.damping_now = None
        sample = stage.GetPrimAtPath(find("dounce_sample"))
        self.sample_scale = next((op for op in UsdGeom.Xformable(sample).GetOrderedXformOps()
                                  if op.GetOpType() == UsdGeom.XformOp.TypeScale), None)
        self.display_error = None
        links = asset_link_paths()
        self.link_names = list(links)
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()] for arm in F.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        # every arm link against each glass part: one column per part
        targets = [f"{actors}/{name}" for name in F.ACTORS]
        self.arm_contacts = self.physics.create_rigid_contact_view(all_links, [targets] * len(all_links))
        # the head and shaft on the bore, its own view
        self.glass_pair = self.physics.create_rigid_contact_view([targets[1]], [[targets[0]]])
        self.tips = [i for i, n in enumerate(self.link_names) if "tip" in n]
        self.ids = self._torch.arange(len(paths), dtype=self._torch.int32, device=self._base.device)

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= F.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):
            raise RuntimeError("Nonfinite glassware state")

    def pose(self, name):
        row = self.poses[self.rows[name]]
        return row[:3].copy(), quat_xyzw_to_matrix(row[3:])

    def measures(self):
        tp, tr = self.pose("tube")
        pp, pr = self.pose("pestle")
        depth, lift, tilt = grading.measure(tp, tr, pp, L.SEAT_Z)
        axis = tr[:, 2]
        v_rel = float((self.velocities[self.rows["pestle"], :3] - self.velocities[self.rows["tube"], :3]) @ axis)
        return {"tube_pos": tp, "tube_rot": tr, "pestle_pos": pp, "pestle_rot": pr, "depth": depth, "tube_lift": lift,
                "tube_tilt_deg": tilt, "v_rel": v_rel, "share": L.submerged_share(depth)}

    def _set_fluid(self, share):
        """The liquid law: damping in proportion to the head's share under the surface. Written only on change."""
        damping = round(L.DRAG * share, 3)
        if damping != self.damping_now:
            self.damping_attr.Set(float(damping))
            self.damping_now = damping

    def _show_liquid(self, depth):
        try:
            if self.sample_scale is not None:
                from pxr import Gf
                self.sample_scale.Set(Gf.Vec3f(1., 1., float(L.surface_height(depth))))
        except Exception as error:       # the drawing is not part of the score
            self.display_error = self.display_error or repr(error)

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        poses = self.body_view.get_transforms().clone()
        for name, start in (("tube", F.TUBE_START), ("pestle", F.PESTLE_START)):
            x, y, z = start + (0., 0., F.TOP + (.0002 if name == "tube" else .0003))
            poses[self.rows[name]] = torch.tensor((x, y, z, 0., 0., 0., 1.), device=poses.device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.ids)
        self._set_fluid(1.)
        observation = super().reset(scene, seed=seed)
        self._read()
        m = self.measures()
        if np.linalg.norm(m["tube_pos"] - F.TUBE_START) > .003 or m["tube_tilt_deg"] > 2.:
            raise RuntimeError(f"Tube is off its well after reset: {m['tube_pos']}, tilt {m['tube_tilt_deg']:.2f} deg")
        if not (L.TOP_BAND < m["depth"] <= L.START_DEPTH + .002) or L.tilt_deg(m["pestle_rot"]) > 5.:
            raise RuntimeError(f"Pestle is not standing in the tube after reset: depth {m['depth']}, "
                               f"tilt {L.tilt_deg(m['pestle_rot']):.2f} deg")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        masses = array(self.body_view.get_masses()).reshape(-1)
        self.masses = {n: round(float(masses[r]), 5) for n, r in self.rows.items()}
        if abs(self.masses["tube"] - L.TUBE_KG) > 5e-4 or abs(self.masses["pestle"] - L.PESTLE_KG) > 5e-4:
            raise RuntimeError(f"Glass masses differ from the scene file: {self.masses}")
        self.grading = grading.DounceGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.sink_check = None
        self._sink_start = None
        self.after_reset = {"tube_mm": [round(1000 * float(v), 2) for v in m["tube_pos"]],
                            "pestle_mm": [round(1000 * float(v), 2) for v in m["pestle_pos"]],
                            "depth_mm": round(1000 * m["depth"], 2), "pestle_tilt_deg": round(L.tilt_deg(m["pestle_rot"]), 2),
                            "fluid_joint": self.fluid_info, "masses_kg": self.masses}
        self._publish_demo_state(m, None)
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self, m, grade):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {k: m[k] for k in ("tube_pos", "tube_rot", "pestle_pos", "pestle_rot", "depth", "tube_lift")}
        self.demo_state["grade"] = grade

    def step(self, action):
        m = self.measures()
        self._set_fluid(m["share"])
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.tick += 1
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        tcps, rots, openings = [], [], []
        for i, arm in ((0, "left"), (7, "right")):
            p, r = ik.tcp_pose(joints[i:i + 6], F.BASES[arm])
            tcps.append(p)
            rots.append(r)
            openings.append(float(joints[i + 6]) * PAD_SPAN_MM)
        m = self.measures()
        grade = self.grading.update({**m, "tcp": tcps})
        self.final_grade = grade
        self._publish_demo_state(m, grade)
        self._show_liquid(m["depth"])
        contacts = self._contacts()
        for key, value in contacts.items():
            peak = float(np.max(np.abs(value))) if np.size(value) else 0.
            self.peaks[key] = round(max(self.peaks.get(key, 0.), peak), 2)
        self._sink(m)
        self.contact_trace.append(self._trace_row(m, grade, contacts, openings))
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(m, grade, tcps, rots, openings, contacts))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == F.MAX_STEPS)
        return StepResult(observation=observation, terminated=success, termination_reason="success" if success else None,
                          info={"dounce_grade": grade})

    def _sink(self, m):
        """Before any hand is near, the pestle sinks under its own 0.2 N at about weight / damping (0.8 mm/s)."""
        lo, hi = SINK_WINDOW
        if self.tick == lo:
            self._sink_start = m["depth"]
        elif self.tick == hi and self._sink_start is not None:
            rate = (self._sink_start - m["depth"]) / ((hi - lo) / F.CONTROL_HZ)
            self.sink_check = {"ticks": [lo, hi], "measured_mm_s": round(1000 * rate, 3),
                               "expected_mm_s": round(1000 * L.PESTLE_KG * 9.81 / L.DRAG, 3),
                               "arms_touching_glass": bool(self.peaks.get("left_arm_on_tube", 0) + self.peaks.get("right_arm_on_pestle", 0) > 0)}

    def _contacts(self):
        """Normal contact forces (N) of the last physics step. Diagnostics only."""
        forces = np.linalg.norm(array(self.arm_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        count = len(forces) // 2
        out = {}
        for arm, rows in (("left", slice(0, count)), ("right", slice(count, 2 * count))):
            block = forces[rows]
            for col, name in enumerate(F.ACTORS):
                out[f"{arm}_arm_on_{name}"] = float(block[:, col].sum())
                out[f"{arm}_tips_on_{name}"] = [float(block[i, col]) for i in self.tips]
        out["pestle_on_tube"] = float(np.linalg.norm(array(self.glass_pair.get_contact_force_matrix(dt=PHYSICS_DT))))
        return out

    def _trace_row(self, m, grade, contacts, openings):
        r = lambda v, k=2: round(float(v), k)
        return {"tick": self.tick, "depth_mm": r(1000 * m["depth"]), "lift_mm": r(1000 * m["tube_lift"]),
                "tilt_deg": r(m["tube_tilt_deg"]), "v_rel_mm_s": r(1000 * m["v_rel"], 1),
                "damping": self.damping_now, "fluid_n_law": r(L.fluid_force(m["v_rel"], m["share"])),
                "left_tips_on_tube_n": [r(v) for v in contacts["left_tips_on_tube"]],
                "right_tips_on_pestle_n": [r(v) for v in contacts["right_tips_on_pestle"]],
                "left_arm_on_pestle_n": r(contacts["left_arm_on_pestle"]), "right_arm_on_tube_n": r(contacts["right_arm_on_tube"]),
                "pestle_on_tube_n": r(contacts["pestle_on_tube"]), "opening_mm": [r(o) for o in openings],
                "score": grade["score"], "strokes": grade["strokes"], "good": grade["good_strokes"]}

    def _diagnose(self, m, grade, tcps, rots, openings, contacts):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        rnd = lambda v, k=1: [round(1000 * float(x), k) for x in v]
        knob = m["pestle_pos"] + m["pestle_rot"][:, 2] * L.KNOB_Z
        grip = m["tube_pos"] + m["tube_rot"][:, 2] * (L.GRIP_Z - L.SEAT_Z)
        row = {"tick": self.tick, **{k: grade[k] for k in grade}, "tube_mm": rnd(m["tube_pos"]), "pestle_mm": rnd(m["pestle_pos"]),
               "pestle_tilt_deg": round(L.tilt_deg(m["pestle_rot"]), 2), "tcp_mm": [rnd(p) for p in tcps],
               "opening_mm": [round(o, 2) for o in openings],
               "tube_in_left_hand_mm": rnd(rots[0].T @ (grip - tcps[0])),
               "handle_in_right_hand_mm": rnd(rots[1].T @ (knob - tcps[1])),
               "display_error": self.display_error}
        row.update({k: ([round(x, 2) for x in v] if isinstance(v, list) else round(v, 2)) for k, v in contacts.items()})
        return row

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class DounceScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import DounceReference

        self.reference = DounceReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the glass parts' poses
        # and the scorer's stroke count), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return DounceArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = DounceScripted()
    policy.info = replace(YamScriptedPolicy().info, name="dounce_scripted")
    return policy
