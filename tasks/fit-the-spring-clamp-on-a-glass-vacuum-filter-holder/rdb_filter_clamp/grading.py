"""Score "fit the spring clamp on a glass vacuum filter holder".

Everything here can be read from a real trial with a ruler, a loupe, the end photo and the trial video: whether
the funnel sits on the base and how far off centre, whether the clamp is closed round both flanges, whether a
hand held the funnel all the while the clamp was being fitted, whether the robot let go and stood back, and,
after the trial, how far the membrane was dragged off its marks (an end-state check).

The membrane is not simulated (it is drawn). The stated rule that stands in for it: whenever the funnel slides
while its ground face rests on the base (faces within 0.3 mm), the membrane under it is carried along with it,
and it stays wherever it is left. Its displacement from where it was laid is read at the end, as a person reads
the ink marks after lifting the funnel off. A drag that later returns to where it started is therefore not
penalised, on the bench or here.
The module needs only NumPy, so the rules can be unit-tested without the simulator.

Order of tiers (each needs the one before):
    0.2  funnel seated on the base (ground faces within 1 mm, axes within 2.5 degrees), but off centre
    0.4  seated and centred: axis within 1.5 mm of the base's
    0.6  clamp on: closed round both flanges (both jaws closed on the bevels), centred within 3 mm, level
    0.8  membrane on its marks (moved no more than 1 mm from where it was laid) and the funnel was held by a
         gripper the whole time the clamp was round the joint and not yet closed on it (watched until the clamp
         has stayed closed on the joint for 0.5 s)
    1.0  the robot has let go and stood back, everything still and the flask upright, for 1 s
Latched 0 for the rest of the trial: the flask knocked over (tilted past 10 degrees) or pushed more than 30 mm
along the table, or the funnel or clamp dropped (funnel tipped past 45 degrees, clamp below 2 cm off the table
away from its start, either on the floor).
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout as L

CONTROL_HZ = 20
SEAT_GAP = .0010                     # ground faces within this of each other at the funnel's lowest point
SEAT_TILT_DEG = 2.5
CENTRED = .0015                      # funnel axis within this of the base's axis
CONTACT_GAP = .0003                  # the funnel carries the membrane only while the ground faces touch
MEMBRANE_LIMIT = .0010               # membrane moved further than this from its marks: dragged (and creased)
LIFTED_GAP = .003                    # funnel lifted this far off the base: no longer seated
CLAMP_CENTRED = .003
CLAMP_HEIGHT = .0025
CLAMP_TILT_DEG = 5.
NEAR_JOINT = .025                    # clamp ring centre within this of the axis (and HEIGHT_BAND of the joint): fitting
HEIGHT_BAND = .015
HOLD_OPENING_MM = (56., 68.)         # a gripper whose pads stopped on the funnel's 64 mm body
HOLD_AXIS = .015                     # its fingertip centre within this of the funnel's axis ...
HOLD_Z = (.026, L.FUNNEL_H)          # ... at a height on the funnel's body
CLEAR_AXIS = .080                    # released: every fingertip centre this far from the holder's axis ...
CLEAR_ABOVE = .050                   # ... or this far above the funnel's top
STILL_M = .001
FITTED_S = .5
HANDS_OFF_S = 1.0
FLASK_TIPPED_DEG, FLASK_UPRIGHT_DEG, FLASK_PUSHED = 10., 3., .030
TIERS = {"seated": .2, "centred": .4, "clamped": .6, "careful": .8, "done": 1.}


def held_by(tcp, rotation, opening_mm, funnel_pos, funnel_rot):
    """Is this gripper holding the funnel's body? From the gripper's joint readings and the funnel's pose."""
    if tcp is None or opening_mm is None:
        return False
    p = np.asarray(funnel_rot, dtype=float).T @ (np.asarray(tcp, dtype=float) - np.asarray(funnel_pos, dtype=float))
    lo, hi = HOLD_OPENING_MM
    return bool(np.hypot(p[0], p[1]) <= HOLD_AXIS and HOLD_Z[0] <= p[2] <= HOLD_Z[1] and lo <= opening_mm <= hi)


@dataclass
class FilterClampGrade:
    holder_start: tuple
    clamp_start: tuple
    failed: str | None = None
    membrane_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))   # membrane's offset from its marks
    touching_xy: np.ndarray | None = None      # the funnel's place on the base at the last step it rested on it
    membrane_peak: float = 0.                  # diagnostics only: the largest offset at any time
    unheld_fitting: bool = False
    fitting_ticks: int = 0
    held_fitting_ticks: int = 0
    clamp_on_ticks: int = 0
    fitted: bool = False                 # the clamp has stayed closed on the joint for FITTED_S: the watch is over
    clear_ticks: int = 0
    history: list = field(default_factory=list)
    last: dict | None = None

    def update(self, s):
        """One 20 Hz tick. ``s`` holds poses in the tabletop frame (positions in m, 3x3 rotations):
        holder_pos, holder_rot, funnel_pos, funnel_rot, clamp_pos, clamp_rot, jaw_open_deg (two values,
        opening positive), and per gripper (lists of two, left then right; None if unreadable): tcp, tcp_rot,
        opening_mm (pads' measured separation)."""
        hp, hr = np.asarray(s["holder_pos"], float), np.asarray(s["holder_rot"], float)
        fp, fr = np.asarray(s["funnel_pos"], float), np.asarray(s["funnel_rot"], float)
        cp, cr = np.asarray(s["clamp_pos"], float), np.asarray(s["clamp_rot"], float)
        seat = L.funnel_on_base(hp, hr, fp, fr)
        ring = L.clamp_on_joint(hp, hr, cp, cr, s["jaw_open_deg"])
        holder_tilt = L.tilt_deg(hr)
        # ---- latched failures ----
        if self.failed is None:
            pushed = float(np.hypot(*(hp[:2] - np.asarray(self.holder_start, float)[:2])))
            clamp_moved = float(np.linalg.norm(cp[:2] - np.asarray(self.clamp_start, float)[:2]))
            if holder_tilt > FLASK_TIPPED_DEG:
                self.failed = "flask knocked over"
            elif pushed > FLASK_PUSHED:
                self.failed = "flask pushed along the table"
            elif L.tilt_deg(fr) > 45. or fp[2] < -.05:
                self.failed = "funnel dropped"
            elif cp[2] < -.05 or (cp[2] < .02 and clamp_moved > .05) or L.tilt_deg(cr) > 60.:
                self.failed = "clamp dropped"
        # ---- the funnel on the base, and the crease rule ----
        seated = seat["gap"] < SEAT_GAP and seat["tilt_deg"] < SEAT_TILT_DEG and seat["offset"] < L.FLANGE_R
        lifted = seat["gap"] > LIFTED_GAP or seat["offset"] >= L.FLANGE_R
        touching = seated and seat["gap"] <= CONTACT_GAP
        if touching:
            if self.touching_xy is not None:      # the membrane goes with the funnel's ground face
                self.membrane_xy = self.membrane_xy + (seat["offset_xy"] - self.touching_xy)
            self.touching_xy = seat["offset_xy"].copy()
        else:
            self.touching_xy = None               # not resting on it: the membrane stays where it is
        membrane = float(np.linalg.norm(self.membrane_xy))
        self.membrane_peak = max(self.membrane_peak, membrane)
        dragged = membrane > MEMBRANE_LIMIT
        centred = seated and seat["offset"] <= CENTRED
        # ---- the clamp ----
        clamp_on = (ring["jaws_closed"] and ring["off_axis"] <= CLAMP_CENTRED and abs(ring["height"]) <= CLAMP_HEIGHT
                    and ring["tilt_deg"] <= CLAMP_TILT_DEG)
        near = ring["off_axis"] <= NEAR_JOINT and abs(ring["height"]) <= HEIGHT_BAND
        tcps, rots, openings = s.get("tcp"), s.get("tcp_rot"), s.get("opening_mm")
        readable = tcps is not None and all(t is not None for t in tcps)
        held = readable and any(held_by(t, r, o, fp, fr) for t, r, o in zip(tcps, rots, openings))
        self.clamp_on_ticks = self.clamp_on_ticks + 1 if clamp_on else 0
        if self.clamp_on_ticks >= round(FITTED_S * CONTROL_HZ):
            self.fitted = True
        fitting = near and not clamp_on and not self.fitted
        if fitting:
            self.fitting_ticks += 1
            if held:
                self.held_fitting_ticks += 1
            else:
                self.unheld_fitting = True
        # ---- hands off ----
        if readable:
            top = fp[2] + L.FUNNEL_H
            clear = all(np.hypot(*(np.asarray(t, float)[:2] - hp[:2])) >= CLEAR_AXIS or t[2] >= top + CLEAR_ABOVE
                        for t in tcps)
        else:
            clear = False                          # unreadable grippers count as still on the work
        self.history = (self.history + [np.r_[fp, cp]])[-(CONTROL_HZ + 1):]
        still = bool(len(self.history) > CONTROL_HZ and
                     np.abs(np.asarray(self.history) - np.r_[fp, cp]).reshape(-1, 2, 3).max() < STILL_M)
        careful = not dragged and not self.unheld_fitting and self.fitting_ticks > 0
        ready = centred and clamp_on and careful and clear and still and holder_tilt <= FLASK_UPRIGHT_DEG
        self.clear_ticks = self.clear_ticks + 1 if ready else 0
        # ---- tier ----
        if self.failed is not None or not seated:
            tier = 0.
        elif not centred:
            tier = TIERS["seated"]
        elif not clamp_on:
            tier = TIERS["centred"]
        elif not careful:
            tier = TIERS["clamped"]
        elif self.clear_ticks >= round(HANDS_OFF_S * CONTROL_HZ):
            tier = TIERS["done"]
        else:
            tier = TIERS["careful"]
        self.last = {
            "score": tier, "failed": self.failed, "seated": bool(seated), "centred": bool(centred),
            "funnel_offset_mm": round(1000 * seat["offset"], 2), "funnel_gap_mm": round(1000 * seat["gap"], 2),
            "funnel_tilt_deg": round(seat["tilt_deg"], 2), "membrane_offset_mm": round(1000 * membrane, 2),
            "membrane_peak_mm": round(1000 * self.membrane_peak, 2), "membrane_dragged": bool(dragged), "clamp_on": bool(clamp_on), "clamp_near": bool(near),
            "clamp_off_axis_mm": round(1000 * ring["off_axis"], 2), "clamp_height_mm": round(1000 * ring["height"], 2),
            "clamp_tilt_deg": round(ring["tilt_deg"], 2), "jaw_open_deg": [round(v, 2) for v in ring["jaw_open_deg"]],
            "held": bool(held), "fitting_ticks": self.fitting_ticks, "held_fitting_ticks": self.held_fitting_ticks,
            "unheld_fitting": self.unheld_fitting, "clear": bool(clear), "still": still,
            "holder_tilt_deg": round(holder_tilt, 2), "clear_ticks": self.clear_ticks,
        }
        return self.last

    @property
    def finished(self):
        return self.last is not None and self.last["score"] == TIERS["done"]


@dataclass(frozen=True)
class FilterClampScorer:
    name: str = "filter_clamp_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        g = record.steps[-1].result.info["filter_clamp_grade"]
        return Score(value=g["score"],
                     explanation=f"seated {g['seated']}, off centre {g['funnel_offset_mm']} mm, clamp on {g['clamp_on']}, "
                                 f"membrane {g['membrane_offset_mm']} mm off its marks, unheld while fitting {g['unheld_fitting']}, "
                                 f"failure {g['failed']}")


def make_scorer():
    return FilterClampScorer()
