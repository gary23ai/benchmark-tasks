"""Score a trial from the cups' poses and the grippers' positions. NumPy only.

Every rule here can be judged from the trial video, a photo of the table and a tape measure:

- apart: no part of either cup is inside the other;
- standing: a cup rests on its base on the table top, within 10 degrees of upright, not moving, with no
  gripper touching it and both gripper centre points at least 5 cm from its surface;
- thrown or dropped: a cup moves through the air more than 5 cm above the table with no gripper centre
  point within 10 cm of it;
- off the table: a cup's centre passes the table edge or drops below the table top.

Score: 0 if a cup was ever thrown, dropped or off the table, or if the cups are not apart at the end.
Otherwise 0.5 for apart, plus 0.25 for each cup that has been standing for the last two seconds.
"""

from dataclasses import dataclass, field

import numpy as np

from . import cup_model as cup

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
UPRIGHT_COS = float(np.cos(np.radians(10.)))
BASE_HEIGHT_RANGE = (-.003, .005)  # base centre above the table top while standing
STILL_LINEAR, STILL_ANGULAR = .010, .15  # m/s, rad/s
CLEAR_OF_GRIPPER = .050  # gripper centre point to cup surface
AIRBORNE_HEIGHT = .050  # lowest point of the cup above the table
ATTENDED_RADIUS = .100  # gripper centre point to the middle of the cup's axis
AIRBORNE_SPEED = .20  # m/s; a cup resting on something is not "moving through the air"
BELOW_TABLE = -.030
AXIS_SAMPLES = np.linspace(0., 1., 5) * cup.HEIGHT


def axis_up(quat_xyzw):
    """The cup's axis (base to rim) as a unit vector."""
    x, y, z, w = quat_xyzw
    return np.array((2. * (x * z + w * y), 2. * (y * z - w * x), 1. - 2. * (x * x + y * y)))


def inside(point, base, up):
    """True when ``point`` is within the solid outline of the cup standing at ``base`` with axis ``up``."""
    offset = point - base
    along = float(offset @ up)
    radial = float(np.linalg.norm(offset - along * up))
    return 0. <= along <= cup.HEIGHT and radial < cup.outside_radius(along)


def nested(pose):
    """True when any point of one cup's axis is inside the other cup."""
    ups = [axis_up(row[3:]) for row in pose]
    for a, b in ((0, 1), (1, 0)):
        if any(inside(pose[a, :3] + s * ups[a], pose[b, :3], ups[b]) for s in AXIS_SAMPLES):
            return True
    return False


def lowest_point(base, up):
    """Height of the lowest point of the cup's base or rim circle."""
    horizontal = float(np.hypot(up[0], up[1]))
    return min(base[2] - cup.BASE_RADIUS * horizontal, base[2] + cup.HEIGHT * up[2] - cup.RIM_RADIUS * horizontal)


def surface_gap(point, base, up):
    """A lower bound on the distance from ``point`` to the cup's surface."""
    offset = point - base
    along = float(np.clip(offset @ up, 0., cup.HEIGHT))
    return float(np.linalg.norm(offset - along * up)) - cup.RIM_RADIUS


@dataclass
class StuckCups:
    """Running grade of one trial. Call :meth:`update` once per control step."""
    table_bounds: tuple = (-.19, .91, -.65, .65)
    ticks: list = field(default_factory=lambda: [0, 0])
    violation: str | None = None

    def update(self, pose, velocity, tcp, touching):
        """``pose``: per cup, base centre XYZ in tabletop coordinates then an XYZW quaternion.
        ``velocity``: per cup, linear then angular velocity. ``tcp``: both gripper centre points.
        ``touching``: per cup, whether any robot part touches it."""
        pose, velocity, tcp = (np.asarray(v, dtype=float) for v in (pose, velocity, tcp))
        if not (np.isfinite(pose).all() and np.isfinite(velocity).all() and np.isfinite(tcp).all()):
            self.violation = self.violation or "invalid simulation state"
            self.ticks = [0, 0]
            return self._grade(False, [False, False], np.nan_to_num(pose))
        ups = [axis_up(row[3:]) for row in pose]
        apart = not nested(pose)
        standing = []
        for i in range(2):
            base, up = pose[i, :3], ups[i]
            centre = base + .5 * cup.HEIGHT * up
            speed = float(np.linalg.norm(velocity[i, :3]))
            x0, x1, y0, y1 = self.table_bounds
            if not (x0 <= centre[0] <= x1 and y0 <= centre[1] <= y1) or base[2] < BELOW_TABLE:
                self.violation = self.violation or f"{cup.CUPS[i]} cup left the table"
            attended = float(np.min(np.linalg.norm(tcp - centre, axis=1))) <= ATTENDED_RADIUS
            if lowest_point(base, up) > AIRBORNE_HEIGHT and speed > AIRBORNE_SPEED and not attended:
                self.violation = self.violation or f"{cup.CUPS[i]} cup was thrown or dropped"
            upright = up[2] >= UPRIGHT_COS and BASE_HEIGHT_RANGE[0] <= base[2] <= BASE_HEIGHT_RANGE[1]
            still = speed <= STILL_LINEAR and float(np.linalg.norm(velocity[i, 3:])) <= STILL_ANGULAR
            clear = not touching[i] and min(surface_gap(p, base, up) for p in tcp) >= CLEAR_OF_GRIPPER
            ok = bool(apart and upright and still and clear)
            self.ticks[i] = self.ticks[i] + 1 if ok else 0
            standing.append(self.ticks[i] >= HOLD_TICKS)
        return self._grade(apart, standing, pose)

    def _grade(self, apart, standing, pose):
        score = 0. if self.violation or not apart else .5 + .25 * sum(standing)
        return {"score": score, "apart": bool(apart), "standing": [bool(s) for s in standing],
                "hold_ticks": list(self.ticks), "violation": self.violation,
                "base_centres_mm": (1000 * pose[:, :3]).round(1).tolist(),
                "base_spacing_mm": round(1000 * float(np.linalg.norm(pose[0, :3] - pose[1, :3])), 1)}


@dataclass(frozen=True)
class StuckCupsScorer:
    name: str = "nested_cups_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["cup_grade"]
        if grade["violation"]:
            text = f'0: {grade["violation"]}'
        elif not grade["apart"]:
            text = "0: the cups are still one inside the other"
        else:
            text = f'cups apart; {sum(grade["standing"])}/2 standing upright and let go for {HOLD_S} s'
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return StuckCupsScorer()
