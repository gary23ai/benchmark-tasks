"""Add the paper stack, the clip and the fixtures to the stock Inspect Robots YAM station, grade every step, run
the lift check and log diagnostics."""

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
from . import paper_model as m

if grading.CONTROL_HZ != fixture.CONTROL_HZ or round(1. / PHYSICS_DT) != fixture.PHYSICS_HZ:
    raise RuntimeError("grading.py, fixed_scene.py and the runtime disagree on the control or physics rate")

BODIES = fixture.ACTORS
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def r1(value):
    return round(float(value), 1)


class PaperArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_paper_jog",
                            environment_id="isaaclab-arena-0.3.1-yam-paper-jog",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a skewed stack of paper on a riser block and a "
                                 "binder clip parked on a small stand. Observations are "
                                 "overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.tick = 0
        self.diagnostics = []
        self.final_grade = None
        self.lift_checks = []
        self.pose_log = None
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
            objects.append(Object(name="paper_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_paper_jog", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one paper fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        links = asset_link_paths()
        self.arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                          for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            self.arm_links["left"], [self.arm_links["right"]] * len(self.arm_links["left"]))
        # One contact view per arm: every link of the arm against every plate and both halves of the clip.
        loose = [f"{actors}/{name}" for name in BODIES]
        self.arm_views = {arm: self.physics.create_rigid_contact_view(sensors, [loose] * len(sensors))
                          for arm, sensors in self.arm_links.items()}

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):
            raise RuntimeError("Nonfinite paper or clip state")

    def pose(self, name):
        row = self.poses[self.rows[name]]
        return row[:3].copy(), quat_xyzw_to_matrix(row[3:])

    def paper_state(self):
        """Every plate's pose (bottom plate first) and the clip, tabletop frame."""
        return {"plates": [self.pose(name) for name in m.PLATES],
                "clip": m.clip_state(*(self.pose(name) for name in m.CLIP_HALVES))}

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        # Every loose body back where the scene file has it: the skewed stack on the block, the clip on its fin.
        poses = self.body_view.get_transforms().clone()
        start = {**m.start_poses(), **m.clip_start()}
        for name, (position, rotation) in start.items():
            row = (*position[:2], position[2] + fixture.TOP, *m.quat_xyzw(rotation))
            poses[self.rows[name]] = torch.tensor(row, device=poses.device, dtype=poses.dtype)
        self.all_ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, self.all_ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), self.all_ids)
        observation = super().reset(scene, seed=seed)
        self._read()
        state = self.paper_state()
        self.state_after_reset = self._state_row(state)
        long_side, short_side = m.misalignment(state["plates"])
        wanted = m.misalignment(list(m.start_poses().values()))
        middle = np.mean([p for p, _ in state["plates"]], axis=0)
        if (abs(long_side - wanted[0]) > .002 or abs(short_side - wanted[1]) > .002
                or np.linalg.norm(middle[:2] - (m.STACK_X, m.STACK_Y)) > .003
                or np.linalg.norm(state["clip"]["origin"] - m.PARK_POS) > .004):
            raise RuntimeError(f"Stack or clip is not at its start after reset: {self.state_after_reset}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        if not np.allclose([masses[self.rows[name]] for name in BODIES], m.MASSES, rtol=.01, atol=0):
            raise RuntimeError("A body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = grading.PaperGrade()
        self.tick = 0
        # Diagnostics only; none of these enters the score.
        self.peak_interarm_n = 0.
        self.peaks = {}
        self.final_grade = None
        self.diagnostics = []
        self.lift_checks = []
        self.demo_state = state
        self.pose_log = [] if os.environ.get("PAPER_JOG_POSE_LOG") else None
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _forces(self):
        """Contact force (N, last physics step) of each arm on the paper (all plates) and on the clip."""
        out = {}
        for arm, view in self.arm_views.items():
            per_body = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum(axis=0)
            out[f"{arm}_on_paper"] = float(per_body[:m.K].sum())
            out[f"{arm}_on_clip"] = float(per_body[m.K:].sum())
        return out

    def _lift_check(self, state):
        """Pull the clip straight up by its back as ``grading.LiftCheck`` asks, then put every body back where it
        was. Extra physics steps after the trial's end state; the robot holds still and no policy step is used."""
        torch = self._torch
        saved = self.body_view.get_transforms().clone(), self.body_view.get_velocities().clone()
        check = grading.LiftCheck(state["plates"], state["clip"], PHYSICS_DT)
        halves = [self.rows[name] for name in m.CLIP_HALVES]
        base, peak, steps = self._base, 0., 0
        while True:
            self._read()
            now = self.paper_state()
            force = check.force(now["plates"], now["clip"], self.velocities[halves, :3].mean(axis=0))
            if force is None:
                break
            peak = max(peak, float(force[2]))
            forces = torch.zeros((len(self.all_ids), 3), dtype=torch.float32, device=self.all_ids.device)
            forces[halves] = torch.tensor(force / 2, dtype=torch.float32, device=self.all_ids.device)
            self.body_view.apply_forces(forces, self.all_ids, is_global=True)
            base.scene.write_data_to_sim()
            base.sim.step(render=False)
            base.scene.update(PHYSICS_DT)
            steps += 1
            if self.recorder is not None and steps % 24 == 0:        # ten review frames a second
                base.sim.render()
                self.recorder.frame()
        held = check.held(now["plates"], now["clip"])
        lowest = min(m.corners(p, r)[:, 2].min() for p, r in now["plates"])
        self.lift_checks.append({"tick": self.tick + 1, "seconds": round(check.time, 2), "hung_clear": check.reached,
                                 "lowest_corner_mm": r1(1000 * lowest), "peak_pull_n": round(peak, 2),
                                 "sheets_held": m.PER_PLATE * int(sum(held)),
                                 "moved_under_lips_mm": [round(1000 * float(v), 2) for v in
                                                         np.linalg.norm(grading.under_lips(now["plates"], now["clip"]) - check.before, axis=1)]})
        self.body_view.set_transforms(saved[0], self.all_ids)
        self.body_view.set_velocities(saved[1], self.all_ids)
        return held

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self._forces()
        for key, value in forces.items():
            self.peaks[key] = max(self.peaks.get(key, 0.), value)
        touching = any(value > grading.TOUCH_N for value in forces.values())
        on_paper = any(forces[f"{arm}_on_paper"] > grading.TOUCH_N for arm in fixture.ARMS)
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        state = self.paper_state()
        last = self.tick + 1 == fixture.MAX_STEPS
        grade = self.grading.update(state["plates"], state["clip"], tcp, touching, on_paper=on_paper, last=last)
        if grade["wants_lift"]:
            grade = self.grading.lifted(grade, self._lift_check(state))
        self.final_grade = grade
        self.tick += 1
        self.demo_state = state
        if self.pose_log is not None:                       # development aid: every body's pose at every step
            self.pose_log.append(self.poses.copy())
        if self.tick % 5 == 0 or grade["score"] == 1.:
            self.diagnostics.append(self._diagnose(joints, state, forces, grade, tcp))
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"paper_grade": grade})

    def _state_row(self, state):
        long_side, short_side = m.misalignment(state["plates"])
        corners = np.array([m.corners(p, r) for p, r in state["plates"]])
        clip = state["clip"]
        return {"long_edges_mm": round(1000 * long_side, 2), "short_edges_mm": round(1000 * short_side, 2),
                "stack_middle_mm": [r1(1000 * v) for v in np.mean([p for p, _ in state["plates"]], axis=0)],
                "lowest_corner_mm": r1(1000 * corners[:, :, 2].min()), "highest_corner_mm": r1(1000 * corners[:, :, 2].max()),
                "largest_angle_deg": round(grading.largest_angle(state["plates"]), 2),
                "clip_origin_mm": [r1(1000 * v) for v in clip["origin"]],
                "clip_opening_deg": round(float(np.degrees(clip["opening"])), 2), "lip_gap_mm": round(1000 * clip["lip_gap"], 2)}

    def _diagnose(self, joints, state, forces, grade, tcp):
        """One row for the run log, in millimetres, degrees and newtons. RESULT.json only; no policy receives it."""
        return {"tick": self.tick, **self._state_row(state),
                "gripper_points_mm": [[r1(1000 * v) for v in point] for point in tcp],
                "gripper_opening": [round(float(joints[6]), 4), round(float(joints[13]), 4)],
                "forces_n": {k: round(v, 2) for k, v in forces.items() if v > 0.},
                "score": grade["score"], "sheets_in_jaws": grade.get("sheets_in_jaws"), "hands_clear": grade.get("hands_clear"),
                "gripper_clearance_mm": grade.get("gripper_clearance_mm"), "hold_ticks": grade["hold_ticks"]}

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        if getattr(self, "pose_log", None):
            np.save(os.environ["PAPER_JOG_POSE_LOG"], np.array(self.pose_log))
        super().close()


class PaperScripted(PolicyBase):
    def __init__(self, variant="full"):
        self.variant = variant

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import PaperReference

        self.reference = PaperReference(self.variant)
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (where the plates and
        # the clip are), which the review rules allow for a scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return PaperArena(**kwargs)


def make_scripted(variant=None):
    from yam_arena.scripted import YamScriptedPolicy

    policy = PaperScripted(variant or os.environ.get("PAPER_JOG_DEMO", "full"))
    policy.info = replace(YamScriptedPolicy().info, name="paper_jog_scripted")
    return policy
