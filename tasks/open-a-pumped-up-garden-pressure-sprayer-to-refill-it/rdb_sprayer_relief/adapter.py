"""Add the pumped-up pressure sprayer on its mat to the stock Inspect Robots YAM station."""

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
from .grading import SprayerGrade

BODIES = fixture.LINKS
VALVE_REST = -L.VALVE_SPRING[0] * L.VALVE_TRAVEL / (L.VALVE_SPRING[1] - L.VALVE_SPRING[0])   # the spring's rest point
CURRENT = None                                       # the live embodiment, for the scripted demonstration


def array(value):
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor.detach().cpu().numpy().copy()


class SprayerArena(YamArenaEmbodiment):
    def __init__(self, *, media_dir=None, start_bar=None):
        global CURRENT
        super().__init__(scene="station", object_jitter=0, control_hz=fixture.CONTROL_HZ,
                         cam_width=320, cam_height=176)
        self.info = replace(self.info, name="yam_sprayer_relief",
                            environment_id="isaaclab-arena-0.3.1-yam-sprayer-relief",
                            supported_target_kinds=frozenset(),
                            docs="Calibrated dual-arm YAM station with a pumped-up 2 litre pressure sprayer standing "
                                 "on a rubber mat. Observations are overhead RGB, both wrist RGB views, and robot "
                                 "joint and jaw positions.")
        self.media_dir = Path(media_dir) if media_dir else None
        self.start_bar = fixture.START_BAR if start_bar is None else float(start_bar)
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
            objects.append(Object(name="sprayer_fixture", object_type=ObjectType.BASE,
                                  spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(fixture.USD))))
            env = IsaacLabArenaEnvironment(name="yam_sprayer_relief", scene=Scene(objects),
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
            raise RuntimeError(f"Expected one sprayer fixture, found {actor_roots}")
        actors = actor_roots[0]
        self.physics = tensors.create_simulation_view("torch")
        self.physics.set_subspace_roots("/")
        self.body_view = self.physics.create_rigid_body_view(actors + "/*")
        paths = list(self.body_view.prim_paths)
        self.rows = {name: paths.index(f"{actors}/{name}") for name in BODIES}
        # The sprayer is one articulation rooted at the tank; its four joints are read from it.
        self.sprayer = self.physics.create_articulation_view(actors + "/tank")
        names = list(self.sprayer.shared_metatype.dof_names)
        self.dofs = {name: names.index(name) for name in fixture.JOINTS}
        links = asset_link_paths()
        arm_links = {arm: [root + "/" + arm.title() + "Arm" + suffix for suffix in links.values()]
                     for arm in fixture.ARMS}
        self.interarm = self.physics.create_rigid_contact_view(
            arm_links["left"], [arm_links["right"]] * len(arm_links["left"]))
        all_links = arm_links["left"] + arm_links["right"]
        self.link_names = list(links)
        targets = [f"{actors}/{name}" for name in BODIES]
        self.fixture_contacts = self.physics.create_rigid_contact_view(all_links, [targets] * len(all_links))

    # -- state ----------------------------------------------------------------------------
    def _read(self):
        self.poses = array(self.body_view.get_transforms())
        self.poses[:, 2] -= fixture.TOP
        self.angles = array(self.sprayer.get_dof_positions()).reshape(-1)
        if not (np.isfinite(self.poses).all() and np.isfinite(self.angles).all()):
            raise RuntimeError("Nonfinite sprayer state")

    def pos(self, name):
        return self.poses[self.rows[name], :3].copy()

    def rotation(self, name):
        return quat_xyzw_to_matrix(self.poses[self.rows[name], 3:])

    def joint(self, name):
        return float(self.angles[self.dofs[name]])

    # -- the one stated rule: tank pressure ---------------------------------------------------
    def _targets(self):
        """Drive target of the valve's spring (its rest point, unchanged). The head and the pump knob have no
        position drive."""
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        targets = torch.zeros_like(self.sprayer.get_dof_positions())
        targets[0, self.dofs["valve"]] = VALVE_REST
        self.sprayer.set_dof_position_targets(targets, one)

    def _pressure(self):
        """One number. It falls while the valve's cap is held out 3 mm or more, and once the head is a quarter
        turn loose (the seal has opened). Nothing in the scene depends on it: no gauge shows it; it is read only by the scorer."""
        self.bar = L.pressure_step(self.bar, self.joint("valve"), self.joint("head"), 1. / fixture.CONTROL_HZ)

    # -- episode --------------------------------------------------------------------------
    def reset(self, scene, *, seed=None):
        from .recording import Recorder

        if self.recorder is not None:
            self.recorder.close()
        self._ensure_sim(None)
        self._physics_views()
        torch = self._torch
        one = torch.arange(1, dtype=torch.int32, device=self._base.device)
        root = self.sprayer.get_root_transforms().clone()
        ox, oy, oz = fixture.ORIGIN
        root[0] = torch.tensor((ox, oy, oz + fixture.TOP + .0005, 0., 0., 0., 1.), device=root.device, dtype=root.dtype)
        self.sprayer.set_root_transforms(root, one)
        self.sprayer.set_root_velocities(torch.zeros_like(self.sprayer.get_root_velocities()), one)
        self.bar = self.start_bar
        start = torch.zeros_like(self.sprayer.get_dof_positions())
        self.sprayer.set_dof_positions(start, one)
        self.sprayer.set_dof_velocities(torch.zeros_like(start), one)
        self._targets()
        observation = super().reset(scene, seed=seed)
        self._read()
        if not np.allclose(self.pos("tank"), fixture.ORIGIN, atol=.004, rtol=0):
            raise RuntimeError(f"Sprayer is off its mark after reset: {self.pos('tank')}")
        if self.rotation("tank")[2, 2] < np.cos(np.radians(2.)):
            raise RuntimeError("Sprayer is not upright after reset")
        if abs(self.joint("head")) > np.radians(2.) or abs(self.joint("valve")) > .0007:
            raise RuntimeError(f"Head is not tight or the valve is not shut after reset: {self.angles}")
        masses = array(self.body_view.get_masses()).reshape(-1)
        if not np.allclose([masses[self.rows[name]] for name in BODIES], fixture.LINK_MASSES, rtol=.01, atol=0):
            raise RuntimeError("A body's mass differs from the declared scene")
        if set(observation.images) != {"top_cam", "left_cam", "right_cam"} or set(observation.state) != {"joint_pos"}:
            raise RuntimeError("Unexpected policy observation fields")
        self.grading = SprayerGrade()
        self.tick = 0
        self.peak_interarm_n = 0.
        self.peak = {"hand_on_valve_n": 0., "hand_on_head_n": 0., "hand_on_knob_n": 0., "hand_on_tank_n": 0.,
                     "tank_yaw_deg": 0., "offset_mm": 0., "tilt_deg": 0.}
        self.final_grade = None
        self.joint_pos = np.asarray(observation.state["joint_pos"], dtype=float).copy()
        self.diagnostics = []
        self._publish_demo_state()
        self.recorder = Recorder(self, self.media_dir) if self.media_dir else None
        if self.recorder is not None:
            self.recorder.start()
        return observation

    def _publish_demo_state(self):
        """Simulator state for the scripted demonstration only. No policy observation contains it."""
        self.demo_state = {"bar": self.bar, "valve": self.joint("valve"), "head": self.joint("head"),
                           "tank_pos": self.pos("tank"), "tank_rot": self.rotation("tank")}

    def tool_points(self, joints):
        return [ik.tcp_pose(joints[i:i + 6], fixture.BASES[arm]) for arm, i in zip(fixture.ARMS, (0, 7), strict=True)]

    def step(self, action):
        self._pressure()
        self._targets()
        observation = super().step(action).observation
        self._read()
        joints = np.asarray(observation.state["joint_pos"], dtype=float)
        self.peak_interarm_n = max(self.peak_interarm_n, float(np.max(np.linalg.norm(
            array(self.interarm.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1))))
        # Contact force of every arm link on each body of the sprayer, columns in BODIES order (diagnostic only).
        forces = np.linalg.norm(array(self.fixture_contacts.get_contact_force_matrix(dt=PHYSICS_DT)), axis=-1)
        self.joint_pos = joints.copy()
        tools = self.tool_points(joints)
        tank = self.rotation("tank")
        grade = self.grading.update(self.bar, self.joint("valve"), self.joint("head"), self.pos("tank") - fixture.ORIGIN,
                                    tank[:, 2], [point - fixture.ORIGIN for point, _ in tools])
        self.final_grade = grade
        self.tick += 1
        self._publish_demo_state()
        row = self._diagnose(joints, tools, forces, grade, tank)
        for name in self.peak:
            value = row[name] if np.isscalar(row[name]) else max(row[name])
            self.peak[name] = max(self.peak[name], abs(float(value)))
        if self.tick % 5 == 0:
            self.diagnostics.append(row)
        success = grade["score"] == 1.
        over = success or self.grading.finished
        if self.recorder is not None:
            self.recorder.step(self.tick, observation, grade, terminal=over or self.tick == fixture.MAX_STEPS)
        reason = "success" if success else grade["void"] if over else None
        return StepResult(observation=observation, terminated=over, termination_reason=reason,
                          info={"sprayer_grade": grade})

    def _diagnose(self, joints, tools, forces, grade, tank):
        """One row for the run log. Written to RESULT.json only; no policy ever receives it."""
        count = len(forces) // 2
        column = {name: i for i, name in enumerate(BODIES)}
        tips = [self.link_names.index(name) for name in ("tip_left", "tip_right") if name in self.link_names]

        def on(body, arm):                       # that arm's two finger tips, then its whole hand and arm
            rows = forces[count * arm:count * (arm + 1), column[body]]
            return [round(float(rows[i]), 2) for i in tips] + [round(float(rows.sum()), 2)]

        return {
            "tick": self.tick,
            "tool_mm": [[round(1000 * float(v), 1) for v in point] for point, _ in tools],     # from joint readings
            "gripper_opening": [round(float(joints[6]), 4), round(float(joints[13]), 4)],
            "bar": grade["bar"],
            "valve_lift_mm": grade["valve_lift_mm"], "head_deg": grade["head_deg"],
            "knob_deg": round(float(np.degrees(self.joint("knob"))), 1),
            "tank_yaw_deg": round(float(np.degrees(np.arctan2(tank[1, 0], tank[0, 0]))), 2),
            "offset_mm": grade["offset_mm"], "tilt_deg": grade["tilt_deg"],
            # [tip, tip, whole arm] newtons for the left arm then the right arm
            "valve_n": on("valve", 0) + on("valve", 1), "head_n": on("head", 0) + on("head", 1),
            "knob_n": on("knob", 0) + on("knob", 1), "tank_n": on("tank", 0) + on("tank", 1),
            "hand_on_valve_n": round(float(forces[:, column["valve"]].sum()), 2),
            "hand_on_head_n": round(float(forces[:, column["head"]].sum()), 2),
            "hand_on_knob_n": round(float(forces[:, column["knob"]].sum()), 2),
            "hand_on_tank_n": round(float(forces[:, column["tank"]].sum()), 2),
            "score": grade["score"],
        }

    def close(self):
        if self.recorder is not None:
            self.recorder.close()
        super().close()


class SprayerScripted(PolicyBase):
    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        from .scripted import SprayerReference

        self.reference = SprayerReference()
        self.actions = self.reference.sequence()

    def act(self, observation):
        # The demonstration plans from the robot's joint readings and from simulator state (the pressure number,
        # the valve's and the head's joint positions, the tank's pose), which the review rules allow for a
        # scripted demonstration.
        self.reference.observe(observation.state["joint_pos"], CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):
    return SprayerArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy

    policy = SprayerScripted()
    policy.info = replace(YamScriptedPolicy().info, name="sprayer_relief_scripted")
    return policy
