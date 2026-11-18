"""Add the loaf cake, its stand and the cutting wire to the stock Inspect Robots YAM station and grade each step."""

from dataclasses import replace
from pathlib import Path
from weakref import WeakValueDictionary

from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.constants import TCP_OFFSET
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix

from . import arm_shapes, cake_model as cake, fixed_scene as fixture, grading

if (grading.CONTROL_HZ, grading.HOLD_S) != (fixture.CONTROL_HZ, fixture.SUCCESS_HOLD_S):
    raise RuntimeError("grading.py and fixed_scene.py disagree on the control rate or hold time")

_RIGS = WeakValueDictionary()      # so that the demonstration can read the simulator it runs in
# PhysX reports contact impulses of the last 240 Hz substep; convert with the physics step.
PHYSICS_DT = 1. / fixture.PHYSICS_HZ
WIRE_ENDS = (slice(0, 4), slice(-4, None))     # nodes that do not collide with their own handle


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


def plain(value):
    """JSON-safe copy of a grade: numpy scalars and arrays become Python numbers and lists."""
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


class CakeArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None):
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_cake_cut",
                            environment_id="isaaclab-arena-0.3.1-yam-cake-layer-wire",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a loaf cake on a stand and a cutting wire "
                                 "with two wooden handles. Observations are overhead RGB, both wrist RGB views, and "
                                 "robot joint and jaw positions.")
        _RIGS[self.info.environment_id] = self
        self.media_dir = Path(media_dir) if media_dir else None
        self.recorder = None
        self.substeps = 0
        self.live = False
        self.wire = fixture.make_wire()

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
            objects.append(Object(name="cake_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_cake_cut", scene=Scene(objects),
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
        # The wire and the uncut cake act at every physics substep: Isaac Lab calls write_data_to_sim()
        # once before each of the 12 PhysX steps inside one 20 Hz control period.
        scene = self._base.scene
        stock_write = scene.write_data_to_sim

        def write_data_to_sim(*args, **kwargs):
            stock_write(*args, **kwargs)
            if self.live:
                self.apply_wire()

        scene.write_data_to_sim = write_data_to_sim

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths

        root = "/World/envs/env_0"
        stage = omni.usd.get_context().get_stage()
        actor_roots = [str(p.GetPath()) for p in stage.Traverse()
                       if p.GetName() == "Actors" and str(p.GetPath()).startswith(root + "/")]
        if len(actor_roots) != 1:
            raise RuntimeError(f"Expected one cake fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in fixture.ACTORS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        targets = [f"{actors}/{name}" for name in fixture.ACTORS]
        # Any arm link against the cake: the cake is to be handled by the wire only.
        all_links = arm_links["left"] + arm_links["right"]
        cake_bodies = [f"{actors}/base", f"{actors}/top_layer"]
        self.cake_contacts = self.physics.create_rigid_contact_view(all_links, [cake_bodies] * len(all_links))
        self.grip_views = {}
        for arm in fixture.ARMS:
            tips = [root + "/" + arm.title() + "Arm" + links[name] for name in ("tip_left", "tip_right")]
            self.grip_views[arm] = self.physics.create_rigid_contact_view(tips, [targets] * 2)
        self.all_ids = self._torch.arange(len(paths), dtype=self._torch.int32, device=self._base.device)
        masses = array(self.body_view.get_masses()).reshape(-1)
        for name in fixture.ACTORS:
            if not np.isclose(masses[self.rows[name]], cake.MASS[name], rtol=.01, atol=1e-4):
                raise RuntimeError(f"{name} mass {masses[self.rows[name]]} differs from the declared {cake.MASS[name]}")
        from .skin import WireSkin
        self.skin = WireSkin(stage, actors.rsplit("/", 1)[0] + "/CuttingWire", fixture.WIRE_SEGMENTS + 1,
                             cake.WIRE_DRAWN_DIAMETER / 2)

    # --- reading the simulator ------------------------------------------------------------------------
    def link_poses(self):
        """Pose of every link of both arms in tabletop coordinates, for keeping the wire out of them."""
        from yam_arena.runtime import link_poses

        out = []
        for arm in fixture.ARMS:
            poses = link_poses(self._base.scene[f"{arm}_robot"])
            out.append({name: (pose[:3, 3] - (0., 0., fixture.TOP), pose[:3, :3]) for name, pose in poses.items()})
        return out

    def check_link_frames(self, joints):
        """The wire is kept out of boxes placed in the links' frames; stop if those frames are not the URDF's."""
        for arm, i, links in zip(fixture.ARMS, (0, 7), self.link_poses(), strict=True):
            position, rotation = links["gripper"]
            expected = ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0]
            found = position + rotation @ np.array(TCP_OFFSET)
            if np.linalg.norm(found - expected) > .005:
                raise RuntimeError(f"{arm} gripper link frame is not the URDF's: centre point {found} vs {expected}")

    def cake_state(self, poses):
        """(pose of the whole cake's centre, pose of the top layer, top layer still in place on the bottom one)."""
        base, top_layer = self.rows["base"], self.rows["top_layer"]
        rotation = quat_xyzw_to_matrix(poses[base, 3:])
        offset = (poses[top_layer, :3] - poses[base, :3]) @ rotation - (0., 0., (cake.BASE_H + cake.TOP_LAYER_H) / 2)
        return ((poses[base, :3] + rotation @ (0., 0., cake.TOP_LAYER_H / 2), rotation),
                (poses[top_layer, :3].copy(), quat_xyzw_to_matrix(poses[top_layer, 3:])),
                bool(np.linalg.norm(offset) < fixture.SEATED))

    def weld(self, poses, velocities):
        """Wrench the uncut cake puts on the top_layer, about the top layer's centre; the bottom layer takes the opposite."""
        on_line, _ = self.wire.cut_fraction()
        intact = 0. if on_line >= grading.THROUGH else 1. - on_line
        self.intact = intact
        if intact == 0.:
            return np.zeros(3), np.zeros(3), np.zeros(3)
        base, top_layer = self.rows["base"], self.rows["top_layer"]
        rb, rs = quat_xyzw_to_matrix(poses[base, 3:]), quat_xyzw_to_matrix(poses[top_layer, 3:])
        arm_b, arm_s = rb @ (0., 0., cake.BASE_H / 2), rs @ (0., 0., -cake.TOP_LAYER_H / 2)
        gap = poses[top_layer, :3] + arm_s - poses[base, :3] - arm_b
        rate = (velocities[top_layer, :3] + np.cross(velocities[top_layer, 3:], arm_s)
                - velocities[base, :3] - np.cross(velocities[base, 3:], arm_b))
        force = -fixture.WELD_STIFFNESS * gap - fixture.WELD_DAMPING * rate
        limit = fixture.WELD_STRENGTH * intact
        force *= min(1., limit / max(np.linalg.norm(force), 1e-9))
        turn = Rotation.from_matrix(rs @ rb.T).as_rotvec()
        torque = (-fixture.WELD_TURN_STIFFNESS * turn
                  - fixture.WELD_TURN_DAMPING * (velocities[top_layer, 3:] - velocities[base, 3:]))
        limit = fixture.WELD_TURN_STRENGTH * intact
        torque *= min(1., limit / max(np.linalg.norm(torque), 1e-9))
        return force, torque, poses[top_layer, :3] + arm_s

    def handle_capsules(self, poses):
        capsules = []
        for arm, skip in zip(fixture.ARMS, WIRE_ENDS, strict=True):
            row = self.rows[f"handle_{arm}"]
            p0, p1 = fixture.handle_capsule(poses[row, :3], quat_xyzw_to_matrix(poses[row, 3:]))
            capsules.append((p0, p1, cake.HANDLE_RADIUS, skip))
        return capsules

    # --- the wire and the uncut cake, every physics step ------------------------------------------------
    def apply_wire(self):
        poses = array(self.body_view.get_transforms())
        poses[:, 2] -= fixture.TOP
        velocities = array(self.body_view.get_velocities())
        forces = np.zeros((len(poses), 3))
        torques = np.zeros_like(forces)
        left, right, base, top_layer = (self.rows[name] for name in ("handle_left", "handle_right", "base", "top_layer"))
        state = self.cake_state(poses)
        boxes = arm_shapes.boxes_near(self.link_poses(), self.wire.x)
        pulls = self.wire.step(poses[left, :3], poses[right, :3], state, fixture.floor, PHYSICS_DT,
                               boxes, self.handle_capsules(poses))
        for arm, row, pull, run in zip(fixture.ARMS, (left, right), pulls[:2], self.wire.taut_run, strict=True):
            forces[row] = fixture.handle_force(pull, run, self.gripped[arm])
            self.peak_handle_force = max(self.peak_handle_force, float(np.linalg.norm(forces[row])))
        # The wire's push on the cake goes to the part it is in; on the scored line both parts share it.
        push = fixture.limited(pulls[2])
        if state[2]:
            local = (self.wire.x - state[0][0]) @ state[0][1]
            share = np.clip(.5 + (local[:, 2] - cake.SEAM_LOCAL) / (2. * fixture.KERF), 0., 1.)
        else:
            share = self.wire.in_top_layer.astype(float)
        for row, part in ((top_layer, share), (base, 1. - share)):
            f = push * part[:, None]
            forces[row] += f.sum(axis=0)
            torques[row] += np.cross(self.wire.x - poses[row, :3], f).sum(axis=0)
        force, torque, at = self.weld(poses, velocities)
        forces[top_layer] += force
        torques[top_layer] += torque + np.cross(at - poses[top_layer, :3], force)
        forces[base] -= force
        torques[base] -= torque + np.cross(at - poses[base, :3], force)
        torch = self._torch
        device = self._base.device
        self.body_view.apply_forces_and_torques_at_position(
            torch.as_tensor(forces.astype(np.float32), device=device),
            torch.as_tensor(torques.astype(np.float32), device=device), None, self.all_ids, True)
        self.substeps += 1
        # Draw the wire where it now is, so that every rendered frame shows it (the first one included).
        self.skin.update(self.wire.x + (0., 0., fixture.TOP))
        self.peak_tension = max(self.peak_tension, float(self.wire.tension.max()))
        self.cake_push = float(np.linalg.norm(push.sum(axis=0)))
        self.peak_cake_push = max(self.peak_cake_push, self.cake_push)

    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.velocities = array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()
                and np.isfinite(self.wire.x).all()):
            raise RuntimeError("Nonfinite cake or wire state")
        self.skin.update(self.wire.x + (0., 0., fixture.TOP))
        for arm in fixture.ARMS:
            forces = array(self.grip_views[arm].get_contact_force_matrix(dt=PHYSICS_DT))
            column = fixture.ACTORS.index(f"handle_{arm}")
            self.gripped[arm] = bool(np.all(np.linalg.norm(forces[:, column], axis=-1) > .10))

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def tension(self):
        """Pull in the wire at its two ends, N."""
        return float(max(self.wire.tension[0], self.wire.tension[-1]))

    def cut(self):
        """Shares of the cake's plan cut on the line and cut anywhere else."""
        return self.wire.cut_fraction()

    def rotation(self, name):
        return quat_xyzw_to_matrix(self.poses[self.rows[name], 3:])

    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self.live = False
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        poses = self.body_view.get_transforms().clone()
        for name, pose in fixture.WORLD_START.items():
            poses[self.rows[name]] = torch.tensor(pose, device=poses.device, dtype=poses.dtype)
        ids = torch.arange(len(self.rows), dtype=torch.int32, device=poses.device)
        self.body_view.set_transforms(poses, ids)
        self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()), ids)
        # Let the slack wire down onto the table before the episode starts.
        capsules = [(*fixture.handle_capsule(cake.HANDLE_START[arm], np.eye(3)), cake.HANDLE_RADIUS, skip)
                    for arm, skip in zip(fixture.ARMS, WIRE_ENDS, strict=True)]
        fixture.lay_wire(self.wire, capsules)
        self.skin.update(self.wire.x + (0., 0., fixture.TOP))     # before the first frame is rendered
        self.peak_tension, self.cake_push, self.peak_cake_push, self.intact = 0., 0., 0., 1.
        self.peak_handle_force = 0.
        self.gripped = dict.fromkeys(fixture.ARMS, False)
        self.live = True
        observation = super().reset(scene, seed=seed)
        self._read()
        for name in fixture.ACTORS:
            if np.linalg.norm(self.pos(name) - cake.START[name]) > .004:
                raise RuntimeError(f"{name} is outside the fixed reset tolerance: {self.pos(name)}")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.check_link_frames(np.asarray(observation.state["joint_pos"], dtype=float))
        self.grading = grading.CakeCutGrade()
        self.tick = 0
        self.substeps = 0
        self.peak_interarm_n = 0.       # a diagnostic; it does not enter the score
        self.peak_cake_touch_n = 0.
        self.final_grade = None
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
        tcp = [ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm])[0] for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]
        poses = {name: self.pos(name) for name in fixture.ACTORS}
        rotations = {name: self.rotation(name) for name in fixture.ACTORS}
        speeds = np.linalg.norm(self.velocities[[self.rows[name] for name in fixture.ACTORS], :3], axis=1)
        touch = array(self.cake_contacts.get_contact_force_matrix(dt=PHYSICS_DT))
        self.peak_cake_touch_n = max(self.peak_cake_touch_n, float(np.max(np.linalg.norm(touch, axis=-1))))
        touching = bool(np.any(np.linalg.norm(touch, axis=-1) > grading.TOUCH_N))
        grade = plain(self.grading.update(touching=touching,
                                          **grading.measure(self.wire, poses, rotations, speeds, tcp, joints[[6, 13]])))
        # Diagnostics only; none of these enters the score.
        grade.update(wire_tension_n=self.tension(), peak_cake_touch_n=self.peak_cake_touch_n,
                     peak_wire_tension_n=self.peak_tension, peak_handle_force_n=self.peak_handle_force,
                     cake_push_n=self.cake_push,
                     peak_cake_push_n=self.peak_cake_push, uncut_bond=self.intact,
                     handles_gripped=[self.gripped[arm] for arm in fixture.ARMS],
                     peak_interarm_n=self.peak_interarm_n)
        self.final_grade = grade
        self.tick += 1
        success = grade["score"] == 1.
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=success or self.tick == fixture.MAX_STEPS)
        return StepResult(observation=observation, terminated=success,
                          termination_reason="success" if success else None, info={"cake_cut_grade": grade})

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class CakeScripted(PolicyBase):
    """The demonstration. It reads the robot's joints and, read-only, the simulator's handle poses, wire tension
    and cut record."""

    def bind(self, embodiment_info):
        self.env = _RIGS[embodiment_info.environment_id]

    def reset(self, scene):
        from .scripted import WireCutReference

        self.reference = WireCutReference(self.env)
        self.actions = self.reference.sequence()

    def act(self, observation):
        self.reference.measured = np.asarray(observation.state["joint_pos"], dtype=float)
        return next(self.actions)


def make_embodiment(**kwargs):
    return CakeArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = CakeScripted()
    policy.info = replace(YamScriptedPolicy().info, name="cake_cut_scripted")
    return policy
