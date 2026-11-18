"""Add the wall, its window and the two-piece magnetic cleaner to the stock Inspect Robots YAM station."""

from dataclasses import replace
from pathlib import Path
from weakref import WeakValueDictionary

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import cleaner_model as model
from . import fixed_scene as fixture
from . import grading
from .magnet import interaction

if (grading.CONTROL_HZ, grading.HOLD_S) != (fixture.CONTROL_HZ, fixture.SUCCESS_HOLD_S):
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate or hold time")

# PhysX reports contact impulses of the last 240 Hz substep; convert with the physics step.
PHYSICS_DT = 1. / 240.
TOUCH_N = .02
_RIGS = WeakValueDictionary()  # lets the scripted demonstration read the simulator's poses of the halves


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class WindowArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_window_wall",
                            environment_id="isaaclab-arena-0.3.1-yam-window-wall",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a wall across the table, an open light "
                                 "and a fixed pane in it, and a two-piece magnetic window cleaner. Observations "
                                 "are overhead RGB, both wrist RGB views, and robot joint and jaw positions.")
        _RIGS[self.info.environment_id] = self
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.magnets_on = False
        self.substeps = 0

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
            objects.append(Object(name="window_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_window_wall", scene=Scene(objects),
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
        # The magnets act at every physics substep: Isaac Lab calls write_data_to_sim()
        # once before each of the 12 PhysX steps inside one 20 Hz control period.
        scene = self._base.scene
        stock_write = scene.write_data_to_sim

        def write_data_to_sim(*args, **kwargs):
            stock_write(*args, **kwargs)
            if self.magnets_on:
                self.apply_magnets()

        scene.write_data_to_sim = write_data_to_sim

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        scopes = {name: [str(p.GetPath()) for p in stage.Traverse()
                         if p.GetName() == name and str(p.GetPath()).startswith(root + "/")]
                  for name in ("Actors", "Marks")}
        if len(scopes["Actors"]) != 1:
            raise RuntimeError(f"Expected one window fixture, found {scopes['Actors']}")
        actors = scopes["Actors"][0]
        self.marks = [stage.GetPrimAtPath(f"{scopes['Marks'][0]}/cell_{row}_{col:02d}")
                      for row in range(grading.GRID_SHAPE[0]) for col in range(grading.GRID_SHAPE[1])] \
            if len(scopes["Marks"]) == 1 else []
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = [paths.index(f"{actors}/{name}") for name in fixture.HALVES]
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        targets = [f"{actors}/{name}" for name in fixture.HALVES]
        self.half_contacts = self.physics.create_rigid_contact_view(all_links, [targets] * len(all_links))
        self.all_ids = self._torch.arange(len(paths), dtype=self._torch.int32, device=self._base.device)

    def apply_magnets(self):
        poses = array(self.body_view.get_transforms())
        a, b = self.rows
        f1, t1, f2, t2 = interaction(poses[a, :3], model.rotation(poses[a, 3:7]),
                                     poses[b, :3], model.rotation(poses[b, 3:7]))
        forces = np.zeros((len(poses), 3), dtype=np.float32)
        torques = np.zeros_like(forces)
        forces[a], torques[a], forces[b], torques[b] = f1, t1, f2, t2
        torch = self._torch
        device = self._base.device
        self.body_view.apply_forces_and_torques_at_position(
            torch.as_tensor(forces, device=device), torch.as_tensor(torques, device=device), None, self.all_ids, True)
        self.substeps += 1
        self.last_magnet_force = float(np.linalg.norm(f2))

    def _read(self):
        self.poses = array(self.body_view.get_transforms())[self.rows]
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())[self.rows]

    def state(self):
        """What the scripted demonstration may read: each half's pad centre and rotation in pane coordinates,
        and whether the scorer currently sees them paired. Policies under test never get this."""
        self._read()
        out = {half: (model.to_pane(self.poses[i, :3]), model.PANE_ROT.T @ model.rotation(self.poses[i, 3:7]))
               for i, half in enumerate(fixture.HALVES)}
        out["paired"] = bool(self.final_grade["paired"]) if self.final_grade else False
        return out

    def _show_line(self, rubbed):
        """Hide the squares of the drawn band that the scorer counts as cleaned. Display only."""
        try:
            from pxr import UsdGeom

            for prim, gone in zip(self.marks, rubbed):
                image = UsdGeom.Imageable(prim)
                if gone:
                    image.MakeInvisible()
                else:
                    image.MakeVisible()
        except Exception as error:  # the drawing is not part of the score
            if not self.line_display_error:
                self.line_display_error = repr(error)

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self.magnets_on = False
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        poses = self.body_view.get_transforms().clone()
        for row, half in zip(self.rows, fixture.HALVES, strict=True):
            poses[row] = torch.tensor(fixture.WORLD_START[half], device=poses.device, dtype=poses.dtype)
        ids = torch.arange(poses.shape[0], dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        self.line_display_error = None
        self._show_line([False] * model.CELLS)
        self.magnets_on = True
        observation = super().reset(scene, seed=seed)
        self._read()
        for i, half in enumerate(fixture.HALVES):
            start = np.array(fixture.WORLD_START[half][:3]) - (0., 0., fixture.TOP)
            if np.linalg.norm(self.poses[i, :3] - start) > .004:
                raise RuntimeError(f"{half} half is outside the fixed reset tolerance: {self.poses[i, :3]}")
        masses = array(self.body_view.get_masses()).reshape(-1)[self.rows]
        if not np.allclose(masses, fixture.RESET_MASS, rtol=0, atol=1e-4):
            raise RuntimeError(f"Cleaner masses differ from the declared scene: {masses}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = grading.WindowGrade()
        self.tick = 0
        self.substeps = 0
        # Diagnostics only; none of these enters the score.
        self.peak_interarm_n = 0.
        self.pull_when_paired_n = None
        self.last_magnet_force = 0.
        self.final_grade = None
        self.trace = []
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = observation.state["joint_pos"]
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        tcp = np.array([ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
                        for arm, i in zip(fixture.ARMS, (0, 7), strict=True)])
        # Rows are robot links, columns are the two halves.
        forces = array(self.half_contacts.get_contact_force_matrix(dt=PHYSICS_DT)).reshape(-1, len(fixture.HALVES), 3)
        touching = (np.linalg.norm(forces, axis=-1) > TOUCH_N).any(axis=0)
        before = self.grading.rubbed.copy()
        grade = self.grading.update(self.poses, self.velocities[:, :3], tcp, touching)
        if not np.array_equal(before, self.grading.rubbed):
            self._show_line(self.grading.rubbed)
        if grade["paired"] and grade["hands_free"]:
            self.pull_when_paired_n = self.last_magnet_force
        grade["magnet_force_n"] = round(self.last_magnet_force, 3)
        # Diagnostics for explaining a failed run, in pane coordinates (u along the wall, n square to it, z up).
        # None of these enters the score.
        half_pane = model.to_pane(self.poses[:, :3])
        backs = [model.PANE_ROT.T @ model.rotation(row[3:7])[:, 2] for row in self.poses]
        grade["tcp_pane_mm"] = (1000. * model.to_pane(tcp)).round(1).tolist()
        grade["half_pane_mm"] = (1000. * half_pane).round(1).tolist()
        grade["half_tilt_deg"] = [round(float(np.degrees(np.arccos(min(1., abs(back[1]))))), 1) for back in backs]
        grade["jaw"] = [round(float(joints[i]), 3) for i in (6, 13)]
        grade["touching"] = [bool(t) for t in touching]
        self.trace.append({key: grade[key] for key in (
            "score", "paired", "violation", "on_glass", "pair_offset_mm", "area_cleaned_fraction", "magnet_force_n",
            "tcp_pane_mm", "half_pane_mm", "half_tilt_deg", "jaw", "touching")})
        self.final_grade = grade
        self.tick += 1
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"magnet_grade": grade})

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class WindowScripted(PolicyBase):
    """The demonstration. It acts only through joint and gripper commands; it reads the observation's joint
    and gripper readings and, from the simulator, the poses of the two halves."""

    env = None

    def bind(self, embodiment_info):
        self.env = _RIGS.get(getattr(embodiment_info, "environment_id", None))

    def reset(self, scene):
        from .scripted import WindowReference

        if self.env is None:  # not bound by the harness: there is only ever one station in a process
            self.env = next(iter(_RIGS.values()), None)
        self.reference = WindowReference(read=self.env.state if self.env is not None else None)
        self.actions = self.reference.sequence()

    def act(self, observation):
        self.reference.measured = np.asarray(observation.state["joint_pos"], dtype=float)
        return next(self.actions)


def make_embodiment(**kwargs):
    return WindowArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = WindowScripted()
    policy.info = replace(YamScriptedPolicy().info, name="window_wall_scripted")
    return policy
