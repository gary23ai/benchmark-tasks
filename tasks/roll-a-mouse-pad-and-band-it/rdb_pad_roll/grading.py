"""Scorer: is the pad a roll, is the band on the roll, is the roll square, and has it been put down and let go.

NumPy only. Everything is judged on the state at the moment of scoring (the end of the trial, or the moment full
marks are reached); nothing is latched. Tiers:

  0     anything else: the pad lies open or is folded rather than rolled, or the band is not on it
  0.3   the pad is a roll and the band is started on it: at least a third of the band's length is over the roll
  0.6   the pad is a roll and the band is on it: the roll goes right through the band and stands at least 10 mm
        out of each of its ends
  1.0   ... and the roll is square (its turns are offset along the roll by 10 mm or less), and both grippers have
        been at least 50 mm from the band and from the pad for 2 s: the banded roll has been put down, wherever,
        and nothing but the band is holding it shut

"A roll": seen from an end, the pad winds one way through at least one full turn, so that its two ends overlap, and
nowhere bends back the other way by more than 20 degrees.
"""

from dataclasses import dataclass

import numpy as np

from . import pad_model as m

CONTROL_HZ = 20
CLEAR_S = 2.0
CLEAR_TICKS = round(CLEAR_S * CONTROL_HZ)
MIN_TURNS = 1.0            # of winding, one way, for the pad to count as rolled: its two ends overlap
BACK_BEND = np.radians(20.)    # a bend this far the other way is a fold
BORE_SLACK = .002          # the bore is a 16-sided prism: its corners are 0.5 mm outside the inscribed circle
PART_ON = 1. / 3.          # of the band's length over the roll, for the lowest tier
BEYOND = .010              # the roll stands this far out of each end of the band, for the band to count as on
SQUARE = .010              # the turns' greatest offset along the roll
HANDS_OFF = .050           # each gripper's centre point this far from the band's surface and from the pad


def fold_angles(rotations):
    """Signed bend (rad) between neighbouring strips about their long axis, from the strips' rotation matrices."""
    out = []
    for a, b in zip(rotations[:-1], rotations[1:]):
        rel = a.T @ b
        out.append(np.arctan2(rel[0, 2], rel[0, 0]))
    return np.array(out)


def strip_edges(centres, rotations):
    """The four long edges of each strip's slab, as their two end points: (strips, 4, 2, 3)."""
    out = []
    for c, r in zip(centres, rotations):
        out.append([[c + r[:, 0] * sx * m.STRIP_W / 2 + r[:, 1] * sy * m.PAD_W / 2 + r[:, 2] * sz * m.PAD_T / 2
                     for sy in (-1, 1)] for sx in (-1, 1) for sz in (-1, 1)])
    return np.array(out)


def point_to_band(point, band_pos, band_rot):
    """Distance from a point to the band's outer surface (0 inside its solid cylinder)."""
    axis = band_rot[:, 0]
    rel = np.asarray(point, dtype=float) - band_pos
    along = float(rel @ axis)
    radial = float(np.linalg.norm(rel - along * axis))
    return float(np.hypot(max(0., abs(along) - m.BAND_L / 2), max(0., radial - m.BAND_OD / 2)))


def point_to_pad(point, strips, strip_rot):
    """Distance from a point to the nearest strip's centre line."""
    point, best = np.asarray(point, dtype=float), np.inf
    for c, r in zip(strips, strip_rot):
        along = float(np.clip((point - c) @ r[:, 1], -m.PAD_W / 2, m.PAD_W / 2))
        best = min(best, float(np.linalg.norm(point - c - along * r[:, 1])))
    return best


class PadRollGrade:
    def __init__(self):
        self.clear = 0             # control steps in a row with both grippers off the band and the pad

    def update(self, *, strips, strip_rot, band_pos, band_rot, tcp):
        strips, strip_rot = np.asarray(strips, dtype=float), np.asarray(strip_rot, dtype=float)
        band_pos, band_rot = np.asarray(band_pos, dtype=float), np.asarray(band_rot, dtype=float)
        axis, half = band_rot[:, 0], m.BAND_L / 2

        # --- is it a roll? ------------------------------------------------------------------------------
        folds = fold_angles(strip_rot)
        net = float(folds.sum())
        way = 1. if net >= 0 else -1.
        turns = abs(net) / (2 * np.pi)
        back = float(max(0., (-way * folds).max()))
        rolled = bool(turns >= MIN_TURNS and back <= BACK_BEND)

        # --- is the band on it? ---------------------------------------------------------------------------
        # Per strip: the part of the band's length that the strip passes through, inside the bore; and how far the
        # strip stands out of the band's two ends. A strip is straight, so within the band it is furthest from the
        # axis at one of the band's ends (or at its own end, if that is inside the band).
        covered, beyond, outside = [], [], 0.
        for edges in strip_edges(strips, strip_rot):
            lo, hi, bore = np.inf, -np.inf, True
            for a, b in edges:
                sa, sb = float((a - band_pos) @ axis), float((b - band_pos) @ axis)
                lo, hi = min(lo, sa, sb), max(hi, sa, sb)
                if abs(sb - sa) < 1e-9 or max(sa, sb) <= -half or min(sa, sb) >= half:
                    continue                                    # this edge does not enter the band
                for s in (np.clip(-half, min(sa, sb), max(sa, sb)), np.clip(half, min(sa, sb), max(sa, sb))):
                    p = a + (s - sa) / (sb - sa) * (b - a) - band_pos
                    radial = float(np.linalg.norm(p - (p @ axis) * axis))
                    outside = max(outside, radial - m.BAND_ID / 2)
                    bore = bore and radial <= m.BAND_ID / 2 + BORE_SLACK
            through = max(0., min(hi, half) - max(lo, -half)) / m.BAND_L
            covered.append(through if bore else 0.)
            beyond.append(min(hi - half, -half - lo) if bore else -np.inf)
        part_on = float(np.mean(covered))
        stands_out = float(min(beyond))
        on = bool(min(covered) > .999 and stands_out >= BEYOND)

        # --- is it square? the turns' offset along the roll (the band's axis once the band is on) --------
        roll_axis = axis if part_on > 0 else np.mean(strip_rot[:, :, 1], axis=0)
        roll_axis = roll_axis / np.linalg.norm(roll_axis)
        offsets = strips @ roll_axis
        telescoped = float(offsets.max() - offsets.min())
        square = bool(telescoped <= SQUARE)

        # --- the grippers away ------------------------------------------------------------------------------
        hands = [min(point_to_band(p, band_pos, band_rot), point_to_pad(p, strips, strip_rot)) for p in tcp]
        self.clear = self.clear + 1 if min(hands) >= HANDS_OFF else 0
        let_go = bool(self.clear >= CLEAR_TICKS)

        score = 0.
        if rolled and part_on >= PART_ON:
            score = .3
            if on:
                score = .6
                if square and let_go:
                    score = 1.
        return {
            "score": score, "pad_is_a_roll": rolled, "band_on_roll": on, "roll_square": square,
            "grippers_clear_2s": let_go,
            "turns": round(turns, 3), "back_bend_deg": round(float(np.degrees(back)), 1),
            "share_of_band_on_roll": round(part_on, 3),
            "roll_out_of_band_mm": round(1000 * max(stands_out, -9.999), 1),
            "outside_bore_mm": round(1000 * max(0., outside), 1),
            "turns_offset_mm": round(1000 * telescoped, 1),
            "gripper_to_band_or_pad_mm": [round(1000 * v, 1) for v in hands],
        }


@dataclass(frozen=True)
class PadRollScorer:
    name: str = "pad_roll_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["pad_roll_grade"]
        return Score(value=grade["score"],
                     explanation=f"Pad is a roll: {grade['pad_is_a_roll']} ({grade['turns']} turns); band on the roll: "
                                 f"{grade['band_on_roll']} (share {grade['share_of_band_on_roll']}); square: "
                                 f"{grade['roll_square']} ({grade['turns_offset_mm']} mm); grippers clear for 2 s: "
                                 f"{grade['grippers_clear_2s']}")


def make_scorer():
    return PadRollScorer()
