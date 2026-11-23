"""Add the carton, the mouse pad, the paper band and its cradle to the stock Inspect Robots YAM station and grade each step."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import pad_model as m

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

PHYSICS_DT = 1. / fixture.PHYSICS_HZ          # PhysX reports contact impulses of the last 240 Hz substep
CURRENT = None                                # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def plain(value):
    """JSON-safe copy: numpy scalars and arrays become Python numbers and lists."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def mm(values):
    return [round(1000 * float(v), 1) for v in values]


LOADED_N = 1.          # a gripper counts as loaded when its fingertips press on the pad or the band this hard


def two_hands(hands, phases, last_tick):
    """From the per-step fingertip loads: the steps on which each gripper, and both at once, were pressing on the
    pad or the band, over the run and for each phase of a scripted demonstration."""
    loads = np.asarray(hands, dtype=float).reshape(-1, 4)
    left, right = loads[:, [0, 2]].max(axis=1), loads[:, [1, 3]].max(axis=1)
    both = (left >= LOADED_N) & (right >= LOADED_N)
    out = {"loaded_means_n": LOADED_N, "steps": len(loads), "left_loaded_steps": int((left >= LOADED_N).sum()),
           "right_loaded_steps": int((right >= LOADED_N).sum()), "both_loaded_steps": int(both.sum()), "by_phase": []}
    starts = [tuple(row) for row in phases or []] + [(last_tick, "end")]
    for (first, name), (after, _) in zip(starts, starts[1:]):
        rows = slice(int(first), int(after))                    # a phase noted at tick t starts with step t + 1
        if len(left[rows]):
            out["by_phase"].append({"phase": name, "steps": int(len(left[rows])), "both_loaded_steps": int(both[rows].sum()),
                                    "left_median_n": round(float(np.median(left[rows])), 2),
                                    "right_median_n": round(float(np.median(right[rows])), 2)})
    return out


class PadArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_pad_roll",
                            environment_id="isaaclab-arena-0.3.1-yam-pad-roll",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a mouse pad lying flat on a carton and an "
                                 "empty paper band in a cradle. Observations are overhead RGB, both wrist RGB "
                                 "views, and robot joint and jaw positions.")
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
            objects.append(Object(name="pad_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_pad_roll", scene=Scene(objects),
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
        stage = omni.usd.get_context().get_stage()
        actor_roots = [str(p.GetPath()) for p in stage.Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected one pad fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        # Free bodies: the pad's strips and links (one floating articulation) and the band.
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in (*fixture.STRIPS, "band")}
        self.strip_rows = [self.rows[name] for name in fixture.STRIPS]
        self.band_ids = self._torch.as_tensor([self.rows["band"]], dtype=self._torch.int32, device=self._base.device)
        self.pad = self.physics.create_articulation_view(f"{actors}/{fixture.ROOT}")
        names = list(self.pad.shared_metatype.dof_names)
        self.fold_dofs = [names.index(name) for name in fixture.FOLDS]
        self.slide_dofs = [names.index(name) for name in fixture.SLIDES]
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        # each fingertip's contact force on each strip of the pad and on the band: a record of which gripper was
        # holding what, and when. It does not enter the score and no policy receives it.
        tips = [root + "/" + arm.title() + "Arm" + links[name] for arm in fixture.ARMS
                for name in ("tip_left", "tip_right")]
        targets = [f"{actors}/{name}" for name in (*fixture.STRIPS, "band")]
        self.fingertips = self.physics.create_rigid_contact_view(tips, [targets] * len(tips))

    # --- reading the simulator ------------------------------------------------------------------------
    def _read(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        dofs = array(self.pad.get_dof_positions()).reshape(-1)
        if not (np.isfinite(poses).all() and np.isfinite(dofs).all()):
            raise RuntimeError("Nonfinite pad or band state")
        self.strips = poses[self.strip_rows, :3].copy()
        self.strip_rot = np.array([quat_xyzw_to_matrix(q) for q in poses[self.strip_rows, 3:]])
        self.folds = dofs[self.fold_dofs]
        self.slides = dofs[self.slide_dofs]
        self.band = poses[self.rows["band"], :3].copy()
        self.band_rot = quat_xyzw_to_matrix(poses[self.rows["band"], 3:])

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"strips": self.strips.copy(), "strip_rot": self.strip_rot.copy(), "folds": self.folds.copy(),
                           "slides": self.slides.copy(), "band": self.band.copy(), "band_rot": self.band_rot.copy()}

    def _place(self):
        """The pad flat on its marks, every hinge and slide at zero; the band in its cradle."""
        torch = self._torch
        device = self._base.device
        one = torch.arange(1, dtype=torch.int32, device=device)
        root = self.pad.get_root_transforms().clone()
        x, y, z = m.PAD_START[m.ROOT]
        root[0] = torch.tensor((x, y, z + fixture.TOP, 0., 0., 0., 1.), device=device, dtype=root.dtype)
        self.pad.set_root_transforms(root, one)
        self.pad.set_root_velocities(torch.zeros_like(self.pad.get_root_velocities()), one)
        zero = torch.zeros_like(self.pad.get_dof_positions())
        self.pad.set_dof_positions(zero, one)
        self.pad.set_dof_velocities(zero, one)
        self.pad.set_dof_position_targets(zero, one)
        poses = self.body_view.get_transforms().clone()
        x, y, z = m.BAND_START
        poses[self.rows["band"]] = torch.tensor((x, y, z + fixture.TOP, 0., 0., 0., 1.), device=device, dtype=poses.dtype)
        self.body_view.set_transforms(poses, self.band_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.band_ids)

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        self._place()
        observation = super().reset(scene, seed=seed)
        self._read()
        # Fail loudly, with the numbers, if the scene did not come up as declared.
        found = {"pad_strips_mm": (1000 * self.strips).round(1).tolist(), "pad_folds_deg": np.degrees(self.folds).round(2).tolist(),
                 "pad_slides_mm": (1000 * self.slides).round(2).tolist(), "band_mm": mm(self.band),
                 "band_axis": self.band_rot[:, 0].round(4).tolist()}
        if np.abs(self.strips - m.PAD_START).max() > .003 or np.abs(self.folds).max() > np.radians(3.):
            raise RuntimeError(f"Pad is not flat on its marks at reset: {found}")
        if np.linalg.norm(self.band - m.BAND_START) > .004 or abs(self.band_rot[0, 0]) < .999:
            raise RuntimeError(f"Band is not in its cradle at reset: {found}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        declared = {"band": m.BAND_MASS, **dict.fromkeys(fixture.STRIPS, m.STRIP_MASS)}
        for name, mass in declared.items():
            if not np.isclose(masses[self.rows[name]], mass, rtol=.02, atol=1e-5):
                raise RuntimeError(f"{name} mass {masses[self.rows[name]]} differs from the declared {mass}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.reset_state = found
        self.grading = grading.PadRollGrade()
        self.tick = 0
        self.peak_interarm_n = 0.       # a diagnostic; it does not enter the score
        self.peak = {"fold_deg": 0., "slide_mm": 0., "turns": 0., "turns_offset_mm": 0.}
        self.final_grade = None
        self.diagnostics = []
        self.hands = []                 # per step: N on the pad from the left gripper, the right; on the band, left, right
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        # rows: left gripper's two fingertips, right gripper's two; columns: the pad's strips, then the band
        tips = np.linalg.norm(array(self.fingertips.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).reshape(4, -1)
        on_pad, on_band = tips[:, :-1].sum(axis=1), tips[:, -1]
        self.hands.append([round(float(v), 2) for v in (on_pad[:2].sum(), on_pad[2:].sum(), on_band[:2].sum(), on_band[2:].sum())])
        tcp = [ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]
        grade = plain(self.grading.update(strips=self.strips, strip_rot=self.strip_rot, band_pos=self.band,
                                          band_rot=self.band_rot, tcp=tcp))
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        self.peak["fold_deg"] = max(self.peak["fold_deg"], float(np.degrees(np.abs(self.folds).max())))
        self.peak["slide_mm"] = max(self.peak["slide_mm"], float(1000 * np.abs(self.slides).max()))
        self.peak["turns"] = max(self.peak["turns"], grade["turns"])
        self.peak["turns_offset_mm"] = max(self.peak["turns_offset_mm"], grade["turns_offset_mm"])
        if self.tick % 5 == 0:
            self.diagnostics.append({**self._diagnose(action, joints, tcp, grade),
                                     "fingertips_on_pad_n": on_pad.round(2).tolist(),        # left gripper's two, right's two
                                     "fingertips_on_band_n": on_band.round(2).tolist()})
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"pad_roll_grade": grade})

    def _diagnose(self, action, joints, tcp, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        command = np.asarray(getattr(action, "data", joints), dtype=float).reshape(-1)
        wanted = [ik.tcp_pose(command[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]
        return {
            "tick": self.tick,
            "gripper_commanded_mm": [mm(p) for p in wanted],              # left, right; from the commanded joints
            "gripper_measured_mm": [mm(p) for p in tcp],                  # from the joint readings
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],     # 1 = 94 mm
            "gripper_opening_commanded": [round(float(command[6]), 3), round(float(command[13]), 3)],
            "pad_first_strip_mm": mm(self.strips[0]), "pad_last_strip_mm": mm(self.strips[-1]),
            "pad_middle_mm": mm(self.strips.mean(axis=0)),
            "pad_folds_deg": np.degrees(self.folds).round(0).tolist(),
            "pad_slides_mm": (1000 * self.slides).round(1).tolist(),
            "band_mm": mm(self.band), "band_axis": self.band_rot[:, 0].round(3).tolist(),
            "turns": grade["turns"], "turns_offset_mm": grade["turns_offset_mm"],
            "share_of_band_on_roll": grade["share_of_band_on_roll"], "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class PadScripted(PolicyBase):
    def __init__(self, variant=None):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import PadReference

        # PAD_DEMO_VARIANT runs one of the negative controls (see scripted.py); unset runs the demonstration.
        self.reference = PadReference(self.variant or os.environ.get("PAD_DEMO_VARIANT") or None)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the pad's strips and
        # the band), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return PadArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = PadScripted()
    policy.info = replace(YamScriptedPolicy().info, name="pad_roll_scripted")
    return policy
