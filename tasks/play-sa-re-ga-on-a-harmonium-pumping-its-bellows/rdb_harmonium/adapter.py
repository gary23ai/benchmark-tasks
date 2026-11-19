"""Add the harmonium to the stock Inspect Robots YAM station, run its air rule and feed the scorer."""

from dataclasses import replace
from pathlib import Path

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from yam_arena import ik
from yam_arena.constants import PHYSICS_DT
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from . import fixed_scene as fixture
from . import layout as L
from .air import Air
from .grading import PhraseGrade

CURRENT = None                                       # the live embodiment, for the scripted demonstration
CONTACT_TRACE_COLUMNS = ("tick", "left_arm_on_flap_n", "left_arm_on_case_n", "left_arm_on_keys_n",
                         "right_arm_on_keys_n", "right_arm_on_case_n", "right_arm_on_flap_n", "flap_rad",
                         "reservoir_l", "pressure_pa", "C4_down_mm", "D4_down_mm", "E4_down_mm", "sounding",
                         "left_tcp_from_edge_mm", "right_tcp_mm")
TOUCH_N = .05                                        # an arm touches the harmonium above this contact force
KEYS_UP_M = .001


def array(value):
    if isinstance(value, np.ndarray):
        return value.copy()
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class HarmoniumArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_harmonium_pump",
                            environment_id="isaaclab-arena-0.3.1-yam-harmonium-pump",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a hand-pumped harmonium, keyboard toward "
                                 "the robot. Observations are overhead RGB, both wrist RGB views, and robot joint "
                                 "and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.demo_state = None
        self.skin = None
        self.tick = 0
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.sound_frames = []
        self.state_after_reset = None
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
            objects.append(Object(name="harmonium_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_harmonium_pump", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one harmonium fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.LINKS}
        self.instrument = self.physics.create_articulation_view(actors + "/case")
        names = list(self.instrument.shared_metatype.dof_names)
        self.dofs = {name: names.index(name) for name in fixture.JOINTS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.link_count = len(links)
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * self.link_count)
        # One contact view per arm: each arm link against every harmonium body (its own column); dt passed on read.
        targets = [f"{actors}/{name}" for name in fixture.LINKS]
        self.arm_contacts = {arm: self.physics.create_rigid_contact_view(arm_links[arm], [targets] * self.link_count)
                             for arm in fixture.ARMS}
        if self.skin is None:
            from .skin import BellowsSkin
            self.skin = BellowsSkin(stage, actors.rsplit("/", 1)[0] + "/BellowsFolds")

    def _read(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        dof = array(self.instrument.get_dof_positions()).reshape(-1)
        if not (np.isfinite(poses).all() and np.isfinite(dof).all()):
            raise RuntimeError("Nonfinite harmonium state")
        self.poses = poses
        self.flap = float(dof[self.dofs[fixture.FLAP_JOINT]])
        self.depths = {name: float(-dof[self.dofs[joint]]) for name, joint in zip(L.KEY_NAMES, fixture.KEY_JOINTS,
                                                                                    strict=True)}

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def rotation(self, name):
        return quat_xyzw_to_matrix(self.poses[self.rows[name], 3:])

    def arm_forces(self):
        """Contact force (N) of each arm on the case, the bellows board and the keys (summed)."""
        out = {}
        for arm, view in self.arm_contacts.items():
            matrix = np.linalg.norm(array(view.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1).sum(axis=0)
            out[arm] = {"case": float(matrix[0]), "flap": float(matrix[1]), "keys": float(matrix[2:].sum())}
        return out

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        root = self.instrument.get_root_transforms().clone()
        root[0] = torch.tensor((*fixture.CASE_START[:2], fixture.TOP, 0., 0., 0., 1.),
                               device=root.device, dtype=root.dtype)
        self.instrument.set_root_transforms(root, one)
        self.instrument.set_root_velocities(torch.zeros_like(self.instrument.get_root_velocities()), one)
        positions = torch.zeros_like(self.instrument.get_dof_positions())
        positions[0, self.dofs[fixture.FLAP_JOINT]] = L.FLAP_OPEN
        self.instrument.set_dof_positions(positions, one)
        self.instrument.set_dof_velocities(torch.zeros_like(positions), one)
        targets = torch.zeros_like(positions)
        for joint in fixture.KEY_JOINTS:
            targets[0, self.dofs[joint]] = L.KEY_PRELOAD
        self.instrument.set_dof_position_targets(targets, one)
        speeds = torch.zeros_like(positions)
        speeds[0, self.dofs[fixture.FLAP_JOINT]] = L.FLAP_OPEN_SPEED
        self.instrument.set_dof_velocity_targets(speeds, one)
        observation = super().reset(scene, seed=seed)
        self._read()
        case = self.pos("case")
        self.state_after_reset = {"case_mm": [round(1000 * float(v), 2) for v in case],
                                  "flap_rad": round(self.flap, 4),
                                  "keys_down_mm_max": round(1000 * max(self.depths.values()), 3)}
        if not np.allclose(case, fixture.CASE_START, atol=.003, rtol=0):
            raise RuntimeError(f"Harmonium is off its marks after reset: {case}")
        # Not fatal: the board's drive reopens it and the key springs lift the keys within half a second, long
        # before the demonstration reaches them; the air rule starts from whatever angle is measured here.
        self.state_after_reset["flap_at_rest"] = abs(self.flap - L.FLAP_OPEN) <= .02
        masses = array(self.body_view.get_masses()).reshape(-1)
        total = float(sum(masses[self.rows[name]] for name in fixture.LINKS))
        if not np.isclose(total, L.MASS, rtol=.01, atol=0) or not np.isclose(masses[self.rows["flap"]], L.FLAP_MASS,
                                                                            rtol=.01, atol=0):
            raise RuntimeError(f"Harmonium masses differ from the declared scene: total {total}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.air = Air()
        self.air.flap = self.flap
        self.pressure = 0.
        self.grading = PhraseGrade(start_xy=tuple(case[:2]))
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak_forces = {"left_arm_on_flap": 0., "left_arm_on_case": 0., "left_arm_on_keys": 0.,
                            "right_arm_on_keys": 0., "right_arm_on_case": 0., "right_arm_on_flap": 0.}
        self.final_grade = None
        self.diagnostics = []
        self.contact_trace = []
        self.sound_frames = []
        self._skin()
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _skin(self):
        try:
            self.skin.update(self.flap, self.pos("case"), self.rotation("case"), fixture.TOP)
        except Exception as error:   # the folds are display only; never let them stop a trial
            self.skin_error = repr(error)

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"flap": self.flap, "volume": self.air.volume, "pressure": self.pressure,
                           "depths": dict(self.depths), "sounding": tuple(self.air.sounding)}

    def step(self, action):
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        forces = self.arm_forces()
        hands_on = any(v > TOUCH_N for arm in forces.values() for v in arm.values())
        self.pressure, sounding = self.air.step(1. / fixture.CONTROL_HZ, self.flap, self.depths)
        self.sound_frames.append((tuple(sounding), self.pressure))
        keys_up = max(self.depths.values()) < KEYS_UP_M
        grade = self.grading.update(sounding, self.pos("case"), self.rotation("case"), hands_on, keys_up,
                                    fed=self.air.fed_m3 > 0.)
        self.final_grade = grade
        self.tick += 1
        self._skin()
        self._publish_demo_state()
        for arm in fixture.ARMS:
            for part in ("flap", "case", "keys"):
                key = f"{arm}_arm_on_{part}"
                self.peak_forces[key] = max(self.peak_forces[key], forces[arm][part])
        self.contact_trace.append([
            self.tick, round(forces["left"]["flap"], 2), round(forces["left"]["case"], 2),
            round(forces["left"]["keys"], 2), round(forces["right"]["keys"], 2), round(forces["right"]["case"], 2),
            round(forces["right"]["flap"], 2), round(self.flap, 4), round(1000 * self.air.volume, 3),
            round(self.pressure, 1), *(round(1000 * self.depths[n], 2) for n in L.PHRASE), "+".join(sounding),
            [round(1000 * float(v), 1) for v in ik.tcp_pose(joints[0:6], fixture.BASES["left"])[0]
             - fixture.flap_corner(self.flap)],
            [round(1000 * float(v), 1) for v in ik.tcp_pose(joints[7:13], fixture.BASES["right"])[0]]])
        if self.tick % 5 == 0:
            self.diagnostics.append(self._diagnose(joints, forces, grade))
        success = grade["score"] == 1.
        over = success or self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = "success" if success else grade["void"] if over else None
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"phrase_grade": grade})

    def _diagnose(self, joints, forces, grade):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        tcp = {}
        for arm, i in zip(fixture.ARMS, (0, 7), strict=True):
            point, _ = ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])
            tcp[arm] = [round(1000 * float(v), 1) for v in point]
        corner = fixture.flap_corner(self.flap)
        return {
            "tick": self.tick,
            "flap_rad": round(self.flap, 4),
            "flap_corner_mm": [round(1000 * float(v), 1) for v in corner],
            "reservoir_l": round(1000 * self.air.volume, 3), "pressure_pa": round(self.pressure, 1),
            "fed_l": round(1000 * self.air.fed_m3, 3), "vented_l": round(1000 * self.air.vented_m3, 3),
            "strokes": self.air.strokes,
            "keys_down_mm": {n: round(1000 * d, 2) for n, d in self.depths.items() if d > .0003},
            "sounding": list(self.air.sounding),
            "tcp_mm": tcp,
            "left_tcp_from_corner_mm": [round(a - 1000 * float(b), 1) for a, b in zip(tcp["left"], corner, strict=True)],
            "right_tcp_from_C4_top_mm": [round(a - 1000 * float(b), 1)
                                         for a, b in zip(tcp["right"], fixture.key_top("C4"), strict=True)],
            "gripper_opening": [round(float(joints[6]), 3), round(float(joints[13]), 3)],
            "contact_n": {arm: {k: round(v, 2) for k, v in forces[arm].items()} for arm in fixture.ARMS},
            "case_moved_mm": grade["moved_mm"], "case_tilt_max_deg": grade["tilt_max_deg"],
            "longest_s": grade["longest_s"], "joins_s": grade["joins_s"], "played": grade["played"],
            "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
            self._write_sound()
        super().close()

    def _write_sound(self):
        """The reeds' sound synthesized from the simulated sounding keys and pressure, plus the calibration
        recording the real-trial rule needs (both synthetic: the simulator has no acoustics)."""
        try:
            from .audio import calibration_recording, synthesize, write_wav

            write_wav(self.media_dir / "sound.wav", synthesize(self.sound_frames))
            write_wav(self.media_dir / "calibration.wav", calibration_recording())
        except Exception as error:   # evidence only; never hide the trial's own result
            self.sound_error = repr(error)


class HarmoniumScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import HarmoniumReference

        self.reference = HarmoniumReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the bellows board's
        # angle, the reservoir's air, key depths and sounding reeds), which the review rules allow.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return HarmoniumArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = HarmoniumScripted()
    policy.info = replace(YamScriptedPolicy().info, name="harmonium_scripted")
    return policy
