"""Score opening the pumped-up sprayer: 0.3 pressure let out first, 0.6 spray head half a turn loose, 0.8 head
unscrewed all the way, 1.0 sprayer left standing on its mark and let go. Loosening the head more than 10 degrees
while the tank is still under pressure is a hazard: 0 for the rest of the trial.

Everything here can be read from the trial video (with sound) and the test piece: the safety valve held open
until it stops venting, the white marks on the head's collar and the tank's neck, whether the head lifts off, where the sprayer stands. The module needs
only NumPy, so the rules can be unit-tested without the simulator.
"""

from dataclasses import dataclass

import numpy as np

from . import layout as L

CONTROL_HZ = 20
HAZARD_TURN_RAD = float(L.SEAL_OPEN)          # the head turned 10 degrees loose ...
SAFE_BAR = float(L.SAFE_BAR)                  # ... with more than this in the tank: the hazard
HALF_TURN_RAD = float(np.pi)
FREE_RAD = float(L.HEAD_FREE - np.radians(10.))   # "unscrewed all the way": within 10 degrees of the thread's end
VALVE_SHUT_M = .001                           # the valve has been let go: its cap is back within 1 mm of shut
MARK_M = .030                                 # the sprayer still stands within this of its mark ...
UPRIGHT_RAD = float(np.radians(5.))           # ... and within this of upright
FALLEN_RAD = float(np.radians(30.))           # tipped this far at any time: knocked over, 0 for the trial
CLEAR_M = .130                                # "let go": both grippers' tool points this far from the sprayer's axis
REST_TICKS = CONTROL_HZ                       # ... and nothing moving for one second
REST_MOVE_M = .0005                           # "still": the sprayer moves less than this in one 50 ms tick
REST_TURN_RAD = float(np.radians(.5))         # and the head turns less than this


def axis_gap(point, base, up):
    """Distance from ``point`` to the sprayer's centre line (base centre to the top of the pump knob)."""
    point, base, up = (np.asarray(v, dtype=float) for v in (point, base, up))
    along = float(np.clip((point - base) @ up, 0., L.HEIGHT))
    return float(np.linalg.norm(point - (base + up * along)))


@dataclass
class SprayerGrade:
    hazard: bool = False                 # the head came a quarter turn loose under pressure (latched)
    fallen: bool = False                 # the sprayer was knocked over (latched)
    vented_first: bool = False           # the pressure was down before the head was a quarter turn loose (latched)
    rest_ticks: int = 0
    previous: tuple | None = None

    def update(self, bar, valve_lift, head_angle, base, up, tool_points):
        """One 20 Hz tick, tabletop frame.

        bar: tank pressure. valve_lift: how far the safety valve's cap is pulled out, metres.
        head_angle: how far the spray head is unscrewed from tight, radians (anticlockwise seen from above).
        base, up: the middle of the tank's base relative to its mark on the mat (metres, z = height above the
        mat) and the sprayer's axis (unit vector).
        tool_points: the two grippers' tool points in the same frame, or None when they cannot be read (then
        nothing counts as let go).
        """
        base, up = np.asarray(base, dtype=float), np.asarray(up, dtype=float)
        tilt = float(np.arccos(np.clip(up[2], -1., 1.)))
        if tilt > FALLEN_RAD:
            self.fallen = True
        if head_angle >= HAZARD_TURN_RAD and bar > SAFE_BAR:
            self.hazard = True
        safe = bar <= SAFE_BAR
        if safe and not self.hazard:
            self.vented_first = True

        offset = float(np.linalg.norm(base[:2]))
        standing = offset <= MARK_M and tilt <= UPRIGHT_RAD
        valve_shut = valve_lift <= VALVE_SHUT_M
        clear = tool_points is not None and all(axis_gap(point, base, up) >= CLEAR_M for point in tool_points)
        now = (base.copy(), float(head_angle))
        still = (self.previous is not None and float(np.linalg.norm(now[0] - self.previous[0])) < REST_MOVE_M
                 and abs(now[1] - self.previous[1]) < REST_TURN_RAD)
        self.previous = now
        free = head_angle >= FREE_RAD
        self.rest_ticks = self.rest_ticks + 1 if (free and standing and valve_shut and clear and still) else 0

        if self.hazard or self.fallen or not self.vented_first:
            score = 0.
        elif head_angle < HALF_TURN_RAD:
            score = .3
        elif not free:
            score = .6
        elif self.rest_ticks >= REST_TICKS:
            score = 1.
        else:
            score = .8
        void = ("the spray head was loosened under pressure" if self.hazard
                else "the sprayer was knocked over" if self.fallen else None)
        return {"score": score, "bar": round(float(bar), 3), "safe": bool(safe), "vented_first": self.vented_first,
                "hazard": self.hazard, "fallen": self.fallen,
                "valve_lift_mm": round(1000 * float(valve_lift), 2), "valve_shut": bool(valve_shut),
                "head_deg": round(float(np.degrees(head_angle)), 1), "head_free": bool(free),
                "offset_mm": round(1000 * offset, 1), "tilt_deg": round(float(np.degrees(tilt)), 2),
                "standing": bool(standing), "grippers_clear": bool(clear), "rest_ticks": self.rest_ticks, "void": void}

    @property
    def finished(self):
        """Nothing more can change the score."""
        return self.hazard or self.fallen


@dataclass(frozen=True)
class SprayerScorer:
    name: str = "sprayer_relief_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["sprayer_grade"]
        return Score(value=grade["score"],
                     explanation=f"Pressure {grade['bar']} bar, let out first: {grade['vented_first']}; head "
                                 f"{grade['head_deg']} degrees loose; standing on its mark: {grade['standing']}; let go: "
                                 f"{grade['grippers_clear']}; void: {grade['void']}")


def make_scorer():
    return SprayerScorer()
