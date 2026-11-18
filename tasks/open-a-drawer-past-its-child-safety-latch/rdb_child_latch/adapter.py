"""Add the storage drawer with its child-safety latch to the stock Inspect Robots YAM station."""

from dataclasses import replace
import os
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import fixed_scene as fixture
from . import grading
from . import layout
from .scripted import fingertips

if grading.CONTROL_HZ != fixture.CONTROL_HZ:
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate")

BODIES = fixture.ACTORS                              # ("drawer", "latch")
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def latch_angle(quat_xyzw):
    """How far the latch arm is pressed down from its stop (rad), from its orientation."""
    axis = quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))[:, 0]
    return float(np.arctan2(-axis[2], axis[0]))


class LatchArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_child_latch_drawer",
                            environment_id="isaaclab-arena-0.3.1-yam-child-latch-drawer",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a plastic storage drawer, shut, fitted "
                                 "with a spring-loaded child-safety latch that stops it after 38 mm. "
                                 "Observations are overhead RGB, both wrist RGB views, and robot joint and "
                                 "jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.diagnostics = []
        self.trace = []
        self.setup_problems = []
        self.tick = 0
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
            objects.append(Object(name="drawer_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_child_latch_drawer", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one drawer fixture, found {actor_roots}")
        actors = actor_roots[0]
        fixture_root = actors.rsplit("/", 1)[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        self.link_names = list(links)
        # One view per pair of interest (a shared view once mislabelled a force for us).
        # Rows: every link of the left arm, then of the right arm; one column, the drawer or the latch.
        self.on_drawer = self.physics.create_rigid_contact_view(all_links, [[f"{actors}/drawer"]] * len(all_links))
        self.on_latch = self.physics.create_rigid_contact_view(all_links, [[f"{actors}/latch"]] * len(all_links))
        # The latch's hook on the fixed catch: the force that holds the drawer. Diagnostics only.
        self.on_catch = None
        self.catch_view_error = None
        try:
            self.on_catch = self.physics.create_rigid_contact_view(
                [f"{actors}/latch"], [[f"{fixture_root}/Static/catch"]])
        except Exception as error:          # a static collider as a filter is not supported everywhere
            self.catch_view_error = repr(error)

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):
            raise RuntimeError("Nonfinite drawer or latch state")

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def drawer_open(self):
        return layout.FRAME_FRONT_X - float(self.pos("drawer")[0])

    def latch_angle(self):
        return latch_angle(self.poses[self.rows["latch"], 3:])

    def latch_pin_offset(self):
        """How far the latch's pin has drifted from where the drawer carries it (m): the joint giving way."""
        return float(np.linalg.norm(self.pos("latch") - layout.hinge_point(self.drawer_open())))

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Every body back where the scene file has it: the drawer shut, the latch arm up against its stop.
        poses = self.body_view.get_transforms().clone()
        for name, (x, y, z) in fixture.WORLD_START.items():
            poses[self.rows[name]] = torch.tensor((x, y, z, 0., 0., 0., 1.), device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        # Checks of the scene as built. Each failure is recorded before the run stops, so RESULT.json says why.
        self.setup_problems = []
        if abs(self.drawer_open() - layout.OPEN_START) > .003:
            self.setup_problems.append(f"drawer not shut after reset: open {1000 * self.drawer_open():.1f} mm")
        if abs(self.latch_angle()) > np.radians(1.5):
            self.setup_problems.append(f"latch arm not resting up on its stop after reset: {np.degrees(self.latch_angle()):.2f} deg "
                                       "(a sign error in the pin's drive or limits would show here)")
        if self.latch_pin_offset() > .002:
            self.setup_problems.append(f"latch pin off its place on the drawer: {1000 * self.latch_pin_offset():.1f} mm")
        masses = array(self.body_view.get_masses()).reshape(-1)
        if not np.allclose([masses[self.rows[name]] for name in BODIES], layout.MASSES, rtol=.01, atol=0):
            self.setup_problems.append("a body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            self.setup_problems.append("unexpected policy observation fields")
        self.latch_angle_after_reset_deg = round(float(np.degrees(self.latch_angle())), 3)
        self.grading = grading.DrawerGrade()
        self.tick = 0
        # Diagnostics only; none of these enters the score.
        self.peak_interarm_n = 0.
        self.peak_arm_on_latch_n = [0., 0.]
        self.peak_arm_on_drawer_n = [0., 0.]
        self.peak_hook_on_catch_n = 0.
        self.peak_open_with_latch_up_mm = 0.
        self.peak_latch_angle_deg = 0.
        self.peak_pin_offset_mm = 0.
        self.first_tick_past_catch = None
        self.final_grade = None
        self.diagnostics = []
        self.trace = []
        self._publish_demo_state()
        if self.setup_problems:
            raise RuntimeError("Scene check failed after reset: " + "; ".join(self.setup_problems))
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"open": self.drawer_open(), "latch_angle": self.latch_angle()}

    def _forces(self, view):
        """Per arm, the summed contact force of its links on the view's body (last physics step)."""
        forces = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).reshape(-1)
        count = len(forces) // 2
        return [float(forces[:count].sum()), float(forces[count:].sum())]

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        on_drawer, on_latch = self._forces(self.on_drawer), self._forces(self.on_latch)
        hook = float("nan")
        if self.on_catch is not None:
            hook = float(np.linalg.norm(array(self.on_catch.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum())
            self.peak_hook_on_catch_n = max(self.peak_hook_on_catch_n, hook)
        touching = bool(max(on_drawer + on_latch) > grading.TOUCH_N)
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        open_m, angle = self.drawer_open(), self.latch_angle()
        grade = self.grading.update(open_m, tcp, touching)
        for i in range(2):
            self.peak_arm_on_latch_n[i] = max(self.peak_arm_on_latch_n[i], on_latch[i])
            self.peak_arm_on_drawer_n[i] = max(self.peak_arm_on_drawer_n[i], on_drawer[i])
        self.peak_latch_angle_deg = max(self.peak_latch_angle_deg, float(np.degrees(angle)))
        self.peak_pin_offset_mm = max(self.peak_pin_offset_mm, 1000. * self.latch_pin_offset())
        if angle < .8 * layout.RELEASE_ANGLE:        # the hook is up: the drawer should stop at 38 mm
            self.peak_open_with_latch_up_mm = max(self.peak_open_with_latch_up_mm, 1000. * open_m)
        self.tick += 1
        if self.first_tick_past_catch is None and open_m >= grading.PAST_M:
            self.first_tick_past_catch = self.tick
        self.final_grade = grade
        self._publish_demo_state()
        # Per-step trace of the quantities that decide the outcome (mm, degrees, N).
        self.trace.append([self.tick, round(1000. * open_m, 2), round(float(np.degrees(angle)), 2),
                           round(on_latch[0], 2), round(on_drawer[1], 2), round(hook, 2) if np.isfinite(hook) else None,
                           round(on_latch[1], 2), round(on_drawer[0], 2)])
        if self.tick % 5 == 0 or grade["score"] == 1.:
            self.diagnostics.append(self._diagnose(joints, on_drawer, on_latch, hook, grade))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"drawer_grade": grade})

    def _diagnose(self, joints, on_drawer, on_latch, hook, grade):
        """One row for the run log, in millimetres, degrees and newtons. Written to RESULT.json only; no policy
        receives it."""
        return {
            "tick": self.tick, "open_mm": grade["open_mm"],
            "drawer_speed_mm_s": round(-1000. * float(self.velocities[self.rows["drawer"], 0]), 1),
            "latch_angle_deg": round(float(np.degrees(self.latch_angle())), 2),
            "latch_pin_offset_mm": round(1000. * self.latch_pin_offset(), 2),
            "hook_on_catch_n": round(hook, 2) if np.isfinite(hook) else None,
            "fingertips_mm": [[round(1000. * float(v), 1) for v in fingertips(joints, arm)] for arm in fixture.ARMS],
            "gripper_opening": [round(float(joints[6]), 4), round(float(joints[13]), 4)],
            "arm_on_latch_n": [round(v, 2) for v in on_latch],
            "arm_on_drawer_n": [round(v, 2) for v in on_drawer],
            "gripper_clearance_mm": grade["gripper_clearance_mm"], "hands_clear": grade["hands_clear"],
            "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class LatchScripted(PolicyBase):
    def __init__(self, variant="full"):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import LatchReference

        self.reference = LatchReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the drawer's opening
        # and the latch arm's angle), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return LatchArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = LatchScripted(variant or os.environ.get("CHILD_LATCH_DEMO", "full"))
    policy.info = replace(YamScriptedPolicy().info, name="child_latch_scripted")
    return policy
