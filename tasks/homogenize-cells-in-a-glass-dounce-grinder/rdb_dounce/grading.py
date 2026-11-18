"""Score "homogenize cells in a glass Dounce grinder".

Everything here can be read from the trial video of a real bench (the side view at the block's height shows the
tube's rim, three tape rings on the pestle's shaft, a line on the tube at the block's top and the liquid) and
from the end state. The module needs only NumPy, so the rules can be unit-tested without the simulator.

Measured each control step (20 Hz):
- depth: how far the pestle's head bottom is above the bore's bottom (from the two glass parts' poses; on a bench,
  which ring is at the rim);
- seated: the tube's bottom within 3 mm of the well's floor and its axis within 5 degrees of vertical;
- covered: the head's top at least 2 mm under the liquid's surface (the red ring stays below the rim);
- speed: the pestle's speed against the tube.
A stroke is a full cycle from the bottom band (head within 5 mm of the bottom) up to the top band (head 27 mm or
more up) and back down to the bottom band; it is counted when the head is back in the bottom band. The first
descent from the start is not a stroke. A stroke is good when, from leaving the bottom band to coming back to it,
the tube stayed seated, the head stayed covered, and the way up and the way down each took at most 4 s (leaving
one band to entering the other).

Tiers:
    0.2, 0.4, 0.6  for 5, 10, 15 good strokes
    0.8  15 good strokes, at most 16 strokes in all, the sample never foamed (the pestle never moved faster than
         35 mm/s against the tube for 0.25 s, the speed at which the up-stroke's suction boils the liquid) and the
         head never came out of the liquid
    1.0  the 0.8 state, then the pestle left with its head on the bottom (within 5 mm), the tube seated, and both
         grippers' fingertip centres at least 30 mm from the tube and the pestle, all held for 1 s
Latched 0 for the rest of the trial: the sample spilled: the tube tipped past 30 degrees or lifted 25 mm, clean out
of its well.
"""

from dataclasses import dataclass

import numpy as np

from . import layout as L

CONTROL_HZ = 20
PASS_S = 4.
FOAM_HOLD_S = .25
CLEAR = .030
STILL_M = .001
HOLD_S = 1.
GOOD_FOR_TIER = 5
STROKES = 15
MAX_STROKES = 16
TIERS = {"five": .2, "ten": .4, "fifteen": .6, "clean": .8, "done": 1.}


def segment_distance(p, a, b):
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0., 1.))
    return float(np.linalg.norm(p - (a + t * ab)))


def measure(tube_pos, tube_rot, pestle_pos, seat_z):
    """depth (m), lift (m), tilt (deg) from the tube's frame origin/rotation and the pestle's frame origin."""
    tube_pos, tube_rot, pestle_pos = (np.asarray(v, dtype=float) for v in (tube_pos, tube_rot, pestle_pos))
    axis = tube_rot[:, 2]
    depth = float((pestle_pos - tube_pos) @ axis) - L.BOTTOM_T
    return depth, float(tube_pos[2] - seat_z), L.tilt_deg(tube_rot)


class DounceGrade:
    def __init__(self, seat_z=L.SEAT_Z):
        self.seat_z = seat_z
        self.tick = 0
        self.failed = None
        self.band = None                # last band the head was in: "bottom" or "top"
        self.visited_top = False        # since the last time the head was in the bottom band
        self.left_bottom = None         # tick of the last step in the bottom band before the current cycle
        self.left_top = None
        self.up_s = None
        self.cycle_ok = True            # seated and covered all through the current cycle
        self.strokes = 0
        self.good = 0
        self.foamed = False
        self.head_out = False
        self.fast_ticks = 0
        self.peak_speed = 0.
        self.peak_lift = 0.
        self.peak_tilt = 0.
        self.prev_depth = None
        self.end_ticks = 0
        self.end_start = None
        self.log = []                   # one row per stroke
        self.last = None

    def update(self, s):
        """``s``: tube_pos, tube_rot, pestle_pos, pestle_rot (tabletop frame) and tcp (list of fingertip centres or
        None when unreadable)."""
        self.tick += 1
        depth, lift, tilt = measure(s["tube_pos"], s["tube_rot"], s["pestle_pos"], self.seat_z)
        seated = lift <= L.SEAT_LIFT and tilt <= L.SEAT_TILT_DEG
        cover = L.head_cover(depth)
        covered = cover >= L.COVER_MIN
        speed = 0. if self.prev_depth is None else abs(depth - self.prev_depth) * CONTROL_HZ
        self.prev_depth = depth
        self.peak_speed = max(self.peak_speed, speed)
        self.peak_lift = max(self.peak_lift, lift)
        self.peak_tilt = max(self.peak_tilt, tilt)
        if tilt > L.SPILL_TILT_DEG or lift > L.OUT_LIFT:
            self.failed = self.failed or ("tube tipped over" if tilt > L.SPILL_TILT_DEG else "tube pulled out of its well")
        if not covered:
            self.head_out = True
        self.fast_ticks = self.fast_ticks + 1 if speed > L.FOAM_SPEED and covered else 0
        if self.fast_ticks >= round(FOAM_HOLD_S * CONTROL_HZ):
            self.foamed = True
        if not (seated and covered):
            self.cycle_ok = False

        band = "bottom" if depth <= L.BOTTOM_BAND else ("top" if depth >= L.TOP_BAND else None)
        if band == "bottom":
            if self.band == "top" and self.visited_top and self.left_bottom is not None:
                down_s = (self.tick - self.left_top) / CONTROL_HZ
                good = self.cycle_ok and self.up_s <= PASS_S and down_s <= PASS_S
                self.strokes += 1
                self.good += int(good)
                self.log.append({"stroke": self.strokes, "tick": self.tick, "good": bool(good), "up_s": self.up_s,
                                 "down_s": down_s, "seated_and_covered": self.cycle_ok})
            self.band = "bottom"
            self.visited_top = False
            self.left_bottom = self.tick              # updated every step in the band: the last one counts
            self.cycle_ok = bool(seated and covered)
        elif band == "top":
            if self.band == "bottom" and not self.visited_top:
                self.up_s = (self.tick - self.left_bottom) / CONTROL_HZ
                self.visited_top = True
            self.band = "top"
            self.left_top = self.tick

        if self.good >= STROKES:
            tier = TIERS["fifteen"]
            if self.strokes <= MAX_STROKES and not self.foamed and not self.head_out:
                tier = TIERS["clean"]
        elif self.good >= 2 * GOOD_FOR_TIER:
            tier = TIERS["ten"]
        elif self.good >= GOOD_FOR_TIER:
            tier = TIERS["five"]
        else:
            tier = 0.
        # the end: pestle on the bottom, tube seated, both hands away, nothing moving
        tcps = s.get("tcp")
        tube_pos, tube_rot = np.asarray(s["tube_pos"], float), np.asarray(s["tube_rot"], float)
        pestle_pos, pestle_rot = np.asarray(s["pestle_pos"], float), np.asarray(s["pestle_rot"], float)
        clear = tcps is not None and all(t is not None for t in tcps) and all(
            segment_distance(t, tube_pos, tube_pos + tube_rot[:, 2] * L.TUBE_L) >= CLEAR
            and segment_distance(t, pestle_pos, pestle_pos + pestle_rot[:, 2] * L.PESTLE_L) >= CLEAR for t in tcps)
        resting = tier == TIERS["clean"] and depth <= L.BOTTOM_BAND and seated and clear
        if resting:
            if self.end_start is None or np.linalg.norm(pestle_pos - self.end_start) > STILL_M:
                self.end_start, self.end_ticks = pestle_pos.copy(), 0
            self.end_ticks += 1
        else:
            self.end_start, self.end_ticks = None, 0
        if resting and self.end_ticks > round(HOLD_S * CONTROL_HZ):
            tier = TIERS["done"]
        if self.failed:
            tier = 0.
        self.last = {
            "score": tier, "failed": self.failed, "depth_mm": round(1000 * depth, 2), "lift_mm": round(1000 * lift, 2),
            "tilt_deg": round(tilt, 2), "seated": bool(seated), "cover_mm": round(1000 * cover, 2),
            "covered": bool(covered), "speed_mm_s": round(1000 * speed, 1), "band": band, "strokes": self.strokes,
            "good_strokes": self.good, "foamed": self.foamed, "head_out": self.head_out, "clear": bool(clear),
            "end_ticks": self.end_ticks, "peak_speed_mm_s": round(1000 * self.peak_speed, 1),
            "peak_lift_mm": round(1000 * self.peak_lift, 2), "peak_tilt_deg": round(self.peak_tilt, 2),
        }
        return self.last

    @property
    def finished(self):
        return self.last is not None and self.last["score"] == TIERS["done"]


@dataclass(frozen=True)
class DounceScorer:
    name: str = "dounce_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        g = record.steps[-1].result.info["dounce_grade"]
        return Score(value=g["score"],
                     explanation=f"{g['good_strokes']} good strokes of {g['strokes']}, foamed {g['foamed']}, head out "
                                 f"{g['head_out']}, peak tube lift {g['peak_lift_mm']} mm, failure {g['failed']}")


def make_scorer():
    return DounceScorer()
