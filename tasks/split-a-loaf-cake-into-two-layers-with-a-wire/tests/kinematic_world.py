"""A stand-in for the simulator, for dry runs of the demonstration and of other strategies.

The arms go where they are told (optionally giving way under the wire's pull like a spring). A handle is
carried rigidly by a gripper that closed on it and drops straight onto the table when released. The wire
and the cut are the real model (rdb_cake_wire.wire), the arm boxes the real ones (rdb_cake_wire.arm_shapes)
and the score the real grader. ``sag`` adds a droop to both shoulders. The cake does not move; the wire's push on it is recorded and compared with
the force that would slide the block. This checks reach, timing, the wire's path and the cut. It says
nothing about PhysX contact: whether the fingers hold a handle, or how the block and top_layer settle.
"""

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN

from rdb_cake_wire import arm_shapes, cake_model as cake, fixed_scene as fixture, grading

EYE = np.eye(3)
PAD_HALF_WIDTH, PAD_REACH = .0077, .013


class KinematicWorld:
    def __init__(self, *, arm_stiffness=None, sag=0., substeps=3):
        self.arm_stiffness, self.sag, self.substeps = arm_stiffness, sag, substeps
        self.wire = fixture.make_wire()
        self.handle = {arm: [cake.HANDLE_START[arm].copy(), EYE.copy()] for arm in fixture.ARMS}
        self.cake = ((cake.CAKE_CENTRE, EYE), (cake.TOP_LAYER_CENTRE, EYE), True)
        capsules = [(*fixture.handle_capsule(*self.handle[arm]), cake.HANDLE_RADIUS, skip)
                    for arm, skip in zip(fixture.ARMS, (slice(0, 4), slice(-4, None)))]
        fixture.lay_wire(self.wire, capsules)              # as the adapter does at reset
        self.held = dict.fromkeys(fixture.ARMS)          # arm -> (handle position, rotation) in the gripper frame
        self.pull = {arm: np.zeros(3) for arm in fixture.ARMS}
        self.give = {arm: np.zeros(3) for arm in fixture.ARMS}
        self.grade = grading.CakeCutGrade()
        self.measured = None
        self.peak = dict(tension=0., push=0., lift=0.)
        self.min_wire_finger_gap = 1.
        self.touched = False
        self.min_over_block = 1.                         # lowest the wire passed above the block's top
        self.log = []

    def advance(self, dt, boxes=()):
        ends = [self.handle[arm][0] for arm in fixture.ARMS]
        capsules = []
        for arm, skip in zip(fixture.ARMS, (slice(0, 4), slice(-4, None))):
            p0, p1 = fixture.handle_capsule(*self.handle[arm])
            capsules.append((p0, p1, cake.HANDLE_RADIUS, skip))
        return self.wire.step(ends[0], ends[1], self.cake, fixture.floor, dt, boxes, capsules)

    # --- what the demonstration may read (the same three calls the simulator adapter offers) -----------
    def pos(self, name):
        return self.handle[name.split("_")[1]][0].copy()

    def tension(self):
        return float(max(self.wire.tension[0], self.wire.tension[-1]))

    def cut(self):
        return self.wire.cut_fraction()

    def step(self, values):
        values = np.asarray(values, dtype=float)
        measured = values.copy()
        links, tcps = [], []
        before = {arm: self.handle[arm][0].copy() for arm in fixture.ARMS}
        for i, arm in enumerate(fixture.ARMS):
            q, jaw = values[7 * i:7 * i + 6].copy(), values[7 * i + 6]
            q[1] += self.sag                                  # the shoulder really droops, and reports it
            measured[7 * i:7 * i + 6] = q
            point, rotation = ik.tcp_pose(q, fixture.BASES[arm])
            if self.arm_stiffness and self.held[arm] is not None:
                # The arm gives way like a damped spring. (Applied a step late, so it is eased in: the full
                # give at once would make this stand-in ring against the wire's own spring; the softer the arm, the slower.)
                ease = min(.1, .25 * self.arm_stiffness / fixture.WIRE_STIFFNESS)
                self.give[arm] += ease * (np.clip(self.pull[arm] / self.arm_stiffness, -.08, .08) - self.give[arm])
                give = self.give[arm]
                q, _, _ = ik.solve(point + give, ik.matrix_to_quat_xyzw(rotation), q, fixture.BASES[arm], iters=60)
                measured[7 * i:7 * i + 6] = q
                point, rotation = ik.tcp_pose(q, fixture.BASES[arm])
            position, turn = self.handle[arm]
            if self.held[arm] is None and jaw < .3:
                local = rotation.T @ (position - point)
                along = abs(rotation[:, 1] @ turn[:, 0])     # the pads' width runs along the gripper's y
                if (abs(local[0]) < .008 and abs(local[1]) < cake.HANDLE_LENGTH / 2 - PAD_HALF_WIDTH
                        and -.020 < local[2] < PAD_REACH - .002 and along > .95):
                    self.held[arm] = (local, rotation.T @ turn)
            elif self.held[arm] is not None and jaw > .5:
                self.held[arm] = None
                axis = turn[:, 0] * (1., 1., 0.)
                axis /= np.linalg.norm(axis)
                self.handle[arm] = [np.array((position[0], position[1], cake.HANDLE_RADIUS)),
                                    np.column_stack((axis, np.cross((0., 0., 1.), axis), (0., 0., 1.)))]
            if self.held[arm] is not None:
                local, local_turn = self.held[arm]
                self.handle[arm] = [point + rotation @ local, rotation @ local_turn]
                measured[7 * i + 6] = cake.HANDLE_RADIUS / abs(FINGER_OPEN)
            links.append(arm_shapes.fk_link_poses(q, measured[7 * i + 6], fixture.BASES[arm]))
            tcps.append(point)
        after = {arm: self.handle[arm][0].copy() for arm in fixture.ARMS}
        boxes = arm_shapes.boxes_near(links, self.wire.x)
        for k in range(1, self.substeps + 1):
            for arm in fixture.ARMS:
                self.handle[arm][0] = before[arm] + k / self.substeps * (after[arm] - before[arm])
            left, right, push = self.advance(1 / (fixture.CONTROL_HZ * self.substeps), boxes)
        self.pull = {"left": left, "right": right}
        total = push.sum(axis=0)
        self.peak["tension"] = max(self.peak["tension"], float(self.wire.tension.max()))
        self.peak["push"] = max(self.peak["push"], float(np.linalg.norm(total[:2])))
        self.peak["lift"] = max(self.peak["lift"], float(total[2]))
        for centre, rotation, half in boxes:
            q = np.abs((self.wire.x[1:-1] - centre) @ rotation) - half
            gap = np.linalg.norm(np.maximum(q, 0.), axis=1) + np.minimum(q.max(axis=1), 0.)
            self.min_wire_finger_gap = min(self.min_wire_finger_gap, float(gap.min()))
        top = cake.CAKE_CENTRE[2] + cake.CAKE_HALF[2]
        over = (np.all(np.abs(self.wire.x[:, :2] - cake.CAKE_CENTRE[:2]) < cake.CAKE_HALF[:2] + .005, axis=1)
                & (self.wire.x[:, 2] > top))
        if over.any():
            self.min_over_block = min(self.min_over_block, float(self.wire.x[over, 2].min() - top))
        dragged = self.peak["push"] > cake.CAKE_SLIDE_FORCE
        poses = {"base": cake.BASE_CENTRE + (.05 * dragged, 0., 0.), "top_layer": cake.TOP_LAYER_CENTRE + (.05 * dragged, 0., 0.)}
        rotations = {"base": EYE, "top_layer": EYE}
        for arm in fixture.ARMS:
            poses[f"handle_{arm}"], rotations[f"handle_{arm}"] = self.handle[arm]
        speeds = [np.linalg.norm(after[arm] - before[arm]) * fixture.CONTROL_HZ for arm in fixture.ARMS]
        touching = False
        for arm_links in links:
            for centre, rotation, half in arm_shapes.boxes_near([arm_links], cake.CAKE_CENTRE[None], reach=.12):
                corners = centre + (np.array(np.meshgrid(*[(-1, 1)] * 3)).T.reshape(-1, 3) * half) @ rotation.T
                touching |= bool(np.any(np.all(np.abs(corners - cake.CAKE_CENTRE) < cake.CAKE_HALF, axis=1)))
        self.touched |= touching
        result = self.grade.update(touching=touching, **grading.measure(self.wire, poses, rotations, speeds, tcps, measured[[6, 13]]))
        self.measured = measured
        self.log.append((float(self.wire.tension.max()), float(np.linalg.norm(total[:2])), result["on_line"],
                         result["off_line"], result["score"]))
        return result


def play(reference, world, max_steps):
    """Run a script in the world until it scores 1.0 or the step limit; returns the last grade and the actions."""
    trace, result = [], None
    for chunk in reference.sequence():
        values = np.asarray(chunk.actions[0].data, dtype=float)
        result = world.step(values)
        reference.measured = world.measured
        trace.append(values)
        if result["score"] == 1. or len(trace) >= max_steps:
            break
    return result, np.array(trace)
