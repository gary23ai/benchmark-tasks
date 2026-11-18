"""Score the layer cut. NumPy only.

Every rule can be judged after a real trial with a ruler, a photo from above and the trial video:

- started: the cut runs at least 60 mm along both long sides of the cake, within 5 mm of the scored line;
- through: the top layer is cut free over its whole area on the line (it lifts off whole, and both layers are
  25 to 35 mm thick);
- spoiled: more than 5 percent of the cake's plan (8 cm2) is cut anywhere else (a second cut, or a cut that
  wandered more than 5 mm off the line);
- touched: a gripper or arm touched the cake at any time (food is handled by the wire only); this is kept for
  the rest of the trial;
- kept: the cake is within 10 mm and 5 degrees of its marks on the stand and the top layer still lies on the
  bottom one, within 10 mm of where it was cut;
- put down: both handles lie on the table top, both grippers are open and their centre points are at least
  5 cm from the handles and from the cake, and nothing is moving, for the last two seconds.

Score: 0 if the cut has not started or the cake was touched. Otherwise 0.25 if the top layer is not cut through
or the cake is spoiled; 0.5 through and not spoiled; 0.75 also kept; 1.0 also put down.
"""

from dataclasses import dataclass

import numpy as np

from . import cake_model as cake

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
STARTED_SIDE_CUT = .060         # m of cut on the line along each long side
THROUGH = .99                   # share of the plan cut on the line
SPOILED = .05                   # share of the plan cut anywhere else
CAKE_SHIFT, CAKE_TURN_DEG = .010, 5.
TOP_LAYER_SHIFT, TOP_LAYER_TILT_DEG, TOP_LAYER_RISE = .010, 5., .005
HANDLE_LYING = cake.HANDLE_RADIUS + .005    # a handle's centre is no higher than this above the table
CLEAR = .050                    # gripper centre point to a handle's axis and to the cake's surface
OPEN_JAW = .80                  # gripper opening, 0 closed .. 1 open
STILL = .020                    # m/s
TOUCH_N = .05                   # N; an arm link pressing on the cake with more than this has touched it


def box_gap(point, centre, rotation, half):
    """Distance from a point to a box's surface (0 inside)."""
    q = np.abs((np.asarray(point) - centre) @ rotation) - half
    return float(np.linalg.norm(np.maximum(q, 0.)))


def segment_gap(point, p0, p1):
    axis = p1 - p0
    t = np.clip((point - p0) @ axis / (axis @ axis), 0., 1.)
    return float(np.linalg.norm(point - p0 - t * axis))


def handle_lying(position):
    """True when a handle rests on the table top itself (not on the stand or the cake)."""
    on_stand = np.all(np.abs(position[:2] - cake.STAND_CENTRE[:2]) < cake.STAND_SIZE[:2] / 2 + cake.HANDLE_RADIUS)
    return bool(-.003 <= position[2] <= HANDLE_LYING and not on_stand)


@dataclass
class CakeCutGrade:
    hold: int = 0
    touched: bool = False

    def update(self, *, on_line, off_line, side_cut, cake_shift, cake_turn_deg, top_layer_shift, top_layer_tilt_deg,
               top_layer_rise, handles_lying, jaws, gripper_gap, speed, touching=False):
        self.touched = self.touched or bool(touching)
        started = min(side_cut) >= STARTED_SIDE_CUT
        through = on_line >= THROUGH
        spoiled = off_line > SPOILED
        kept = (cake_shift <= CAKE_SHIFT and cake_turn_deg <= CAKE_TURN_DEG and top_layer_shift <= TOP_LAYER_SHIFT
                and top_layer_tilt_deg <= TOP_LAYER_TILT_DEG and abs(top_layer_rise) <= TOP_LAYER_RISE)
        put_down = (all(handles_lying) and min(jaws) >= OPEN_JAW and gripper_gap >= CLEAR and speed < STILL)
        self.hold = self.hold + 1 if put_down else 0
        if not started or self.touched:
            score = 0.
        elif spoiled or not through:
            score = .25
        else:
            score = .5 + .25 * kept + .25 * (kept and self.hold >= HOLD_TICKS)
        return {"score": float(score), "started": bool(started), "through": bool(through), "spoiled": bool(spoiled),
                "kept": bool(kept), "touched": bool(self.touched), "put_down": bool(put_down), "hold_ticks": int(self.hold),
                "on_line": float(on_line), "off_line": float(off_line),
                "side_cut_mm": [round(1000. * float(s), 1) for s in side_cut],
                "cake_shift_mm": 1000. * float(cake_shift), "cake_turn_deg": float(cake_turn_deg),
                "top_layer_shift_mm": 1000. * float(top_layer_shift), "top_layer_tilt_deg": float(top_layer_tilt_deg),
                "top_layer_rise_mm": 1000. * float(top_layer_rise), "gripper_gap_mm": 1000. * float(gripper_gap),
                "speed": float(speed)}


def measure(wire, poses, rotations, velocities, tcp, jaws):
    """Everything the grade needs, from the wire's record of the cut and the bodies' poses.

    ``poses`` and ``rotations`` are {name: position} and {name: 3x3 rotation} in tabletop coordinates for the
    four bodies of cake_model.ACTORS; ``velocities`` their speeds (m/s); ``tcp`` the two gripper centre points.
    """
    on_line, off_line = wire.cut_fraction()
    base_r, top_layer_r = rotations["base"], rotations["top_layer"]
    offset = (poses["top_layer"] - poses["base"]) @ base_r - (0., 0., (cake.BASE_H + cake.TOP_LAYER_H) / 2)
    turn = lambda r: float(np.degrees(np.arccos(np.clip((np.trace(r) - 1.) / 2., -1., 1.))))
    block_centre = poses["base"] + base_r @ (0., 0., cake.TOP_LAYER_H / 2)
    gaps = []
    for point in tcp:
        gaps.append(box_gap(point, block_centre, base_r, cake.CAKE_HALF))
        gaps.append(box_gap(point, poses["top_layer"], top_layer_r, (*cake.CAKE_HALF[:2], cake.TOP_LAYER_H / 2)))
        for side in ("left", "right"):
            reach = rotations[f"handle_{side}"][:, 0] * cake.HANDLE_LENGTH / 2
            centre = poses[f"handle_{side}"]
            gaps.append(segment_gap(np.asarray(point), centre - reach, centre + reach))
    return dict(
        on_line=on_line, off_line=off_line, side_cut=wire.side_cut(),
        cake_shift=float(np.linalg.norm((poses["base"] - cake.BASE_CENTRE)[:2])), cake_turn_deg=turn(base_r),
        top_layer_shift=float(np.linalg.norm(offset[:2])), top_layer_tilt_deg=turn(top_layer_r @ base_r.T), top_layer_rise=float(offset[2]),
        handles_lying=[handle_lying(poses[f"handle_{side}"]) for side in ("left", "right")],
        jaws=[float(j) for j in jaws], gripper_gap=min(gaps), speed=float(max(velocities)))


@dataclass(frozen=True)
class CakeCutScorer:
    name: str = "cake_cut_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["cake_cut_grade"]
        return Score(value=grade["score"], explanation=f"Put down and still: {grade['hold_ticks']}/{HOLD_TICKS} ticks")


def make_scorer():
    return CakeCutScorer()
