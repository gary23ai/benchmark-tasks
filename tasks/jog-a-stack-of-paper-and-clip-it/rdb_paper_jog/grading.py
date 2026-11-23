"""Score the trial from where the sheets, the binder clip and the grippers are at the end. NumPy only.

In the simulator the 36 sheets are nine rigid plates of four sheets; a plate counts as four sheets.

Measured at the end of the trial (README, "Scoring a real trial"):

- off the table: any sheet below the table top or beyond its edges. 0.
- let go: for the last 2 s no part of the robot has touched the stack or the clip and both grippers' centre
  points have been at least 50 mm from them.
- in the jaws: a sheet lies between the clip's two lips, the middle of the lips at least 3 mm in from the
  sheet's edge, with the lips shut on the paper (the gap between the lips no more than the thickness of the
  paper they are over, outer face to outer face, plus 1 mm; a clip held open round the stack is not shut).
- lift check (only when let go): the clip is pulled straight up by its back until the whole stack hangs from
  it, its lowest corner 50 mm above the block's top (152 mm above the table), and held 2 s. A sheet is held if
  it is still in the jaws and has moved less than 3 mm under the lips. ``held`` is the number of sheets held;
  without a lift check it is the number in the jaws.
- flush: the largest step between sheet edges, on the long sides and on the short sides (``paper_model.misalignment``).
- clip placed: the clip is on a short edge (its jaws point along the long side within 25 degrees) and the
  middle of its lips is within 20 mm of the middle of that edge.

Score: 1.0 let go, all 36 sheets held through the lift check, both flush values at most 2 mm, clip placed;
0.6 let go, all 36 held through the lift check, both flush values at most 6 mm (so: 2 to 6 mm, or the clip
not placed); 0.3 at least 18 sheets held (all in the jaws but more than 6 mm out of flush, or not let go, or
only some sheets held); else 0.
"""

from dataclasses import dataclass, field

import numpy as np

from . import paper_model as m

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
CLEAR_M = .050                   # gripper centre point to the stack and to the clip
TOUCH_N = .05                    # contact force that counts as touching
FLUSH_M = .002                   # full score
ROUGH_M = .006                   # partial score
JAW_IN_M = .003                  # the lips' middle this far in from the sheet's edge
JAW_SLACK_M = .001               # lips shut on the sheets: gap at most the sheets between them plus this
PLACE_M = .020                   # middle of the lips to the middle of the short edge
PLACE_DEG = 25.
TABLE = ((-.19, .91), (-.65, .65))       # table top, x and y ranges
FELL_M = .030                    # below the table top by this: off the table
SHEETS = m.N_SHEETS
HALF = SHEETS // 2

# the lift check
LIFT_CLEAR_M = .050              # the hanging stack's lowest corner above the block's top
LIFT_Z = m.BLOCK[2] + LIFT_CLEAR_M
LIFT_HOLD_S = 2.
LIFT_SPEED = .15                 # m/s
LIFT_KP, LIFT_KD, LIFT_SIDE_KD = 300., 6., 1.5
LIFT_MAX_N = 12.
LIFT_TIMEOUT_S = 8.
LIFT_SLIP_M = .003


def in_plate(point, pose):
    """``point`` in a plate's frame: (u, v, w)."""
    position, rotation = pose
    return np.asarray(rotation, dtype=float).T @ (np.asarray(point, dtype=float) - np.asarray(position, dtype=float))


def lips_middle(clip):
    return clip["origin"] + m.JAW_L * clip["rotation"][:, 2]


def lip_line(clip, samples=9):
    """Points along the line the two lips close on, from one end of the clip's width to the other."""
    middle = lips_middle(clip)
    return [middle + s * clip["rotation"][:, 1] for s in np.linspace(-m.CLIP_W / 2, m.CLIP_W / 2, samples)]


def in_jaws(poses, clip):
    """Which plates are in the jaws: between the lips with the middle of the lips at least JAW_IN_M in from the
    plate's edge, the lips being shut on the paper: the gap between them no more than the thickness of the paper
    they are over (outer face to outer face of every plate that any part of the lips is over) plus
    JAW_SLACK_M. A clip held open round the stack is not shut on it."""
    line = lip_line(clip)
    middle = len(line) // 2
    inside, across = [], []
    for pose in poses:
        local = np.array([in_plate(point, pose) for point in line])
        margins = np.minimum(m.SHEET_U / 2 - np.abs(local[:, 0]), m.SHEET_V / 2 - np.abs(local[:, 1]))
        between = np.abs(local[:, 2]) <= clip["lip_gap"] / 2 + m.PLATE_T
        if np.any(between & (margins >= 0.)):
            across.append(float(local[middle, 2]))
        inside.append(bool(between[middle] and margins[middle] >= JAW_IN_M))
    shut = bool(across) and clip["lip_gap"] <= max(across) - min(across) + m.PLATE_T + JAW_SLACK_M
    return inside if shut else [False] * len(poses)


def under_lips(poses, clip):
    """Each plate's point under the middle of the lips, in that plate's own frame (u, v)."""
    middle = lips_middle(clip)
    return np.array([in_plate(middle, pose)[:2] for pose in poses])


def clip_placed(poses, clip):
    """(placed, distance of the lips' middle from the middle of the nearest short edge, jaw angle off the long side)."""
    rotations = np.array([np.asarray(r, dtype=float) for _, r in poses])
    v_axis = rotations[:, :, 1].mean(axis=0)
    v_axis /= np.linalg.norm(v_axis)
    off = float(np.degrees(np.arccos(np.clip(abs(clip["rotation"][:, 2] @ v_axis), 0., 1.))))
    along = float(np.mean([in_plate(lips_middle(clip), pose)[0] for pose in poses]))
    return bool(off <= PLACE_DEG and abs(along) <= PLACE_M), abs(along), off


def largest_angle(poses):
    normals = np.array([np.asarray(r, dtype=float)[:, 2] for _, r in poses])
    cos = np.abs(normals @ normals.T).min()
    return float(np.degrees(np.arccos(np.clip(cos, 0., 1.))))


def fell(poses):
    for position, rotation in poses:
        pts = m.corners(position, rotation)
        if (pts[:, 2].min() < -FELL_M or pts[:, 0].min() < TABLE[0][0] or pts[:, 0].max() > TABLE[0][1]
                or pts[:, 1].min() < TABLE[1][0] or pts[:, 1].max() > TABLE[1][1]):
            return True
    return False


def clearance(tcp, poses, clip):
    """Smallest distance from a gripper centre point to the stack (any plate) or to the clip."""
    best = np.inf
    half = np.array((m.SHEET_U / 2, m.SHEET_V / 2, m.PLATE_T / 2))
    reach = np.array((m.BACK_HALF + m.HANDLE_L * np.tan(m.HANDLE_SPLAY) + .002, m.CLIP_W / 2, 0.))
    low, high = -reach - (0., 0., m.HANDLE_L), reach + (0., 0., m.JAW_L + m.LIP_R)
    for point in np.atleast_2d(np.asarray(tcp, dtype=float)):
        for pose in poses:
            local = in_plate(point, pose)
            best = min(best, float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.))))
        local = clip["rotation"].T @ (point - clip["origin"])
        best = min(best, float(np.linalg.norm(np.maximum(np.maximum(low - local, local - high), 0.))))
    return best


class LiftCheck:
    """Pulls the clip straight up by its back, like a hand lifting the stack by the clip, and decides which
    sheets it carried. Call :meth:`force` every physics step and apply the force (N, tabletop frame) to the
    clip, spread over its two halves; stop when it returns None, then read :meth:`held`."""

    def __init__(self, poses, clip, dt):
        self.dt = dt
        self.time = 0.
        self.held_for = 0.
        self.target = float(clip["origin"][2])
        self.before = under_lips(poses, clip)
        self.was_in = in_jaws(poses, clip)
        self.done = False
        self.reached = False

    def force(self, poses, clip, clip_velocity):
        lowest = min(m.corners(p, r)[:, 2].min() for p, r in poses)
        self.time += self.dt
        if lowest >= LIFT_Z:
            self.reached = True
            self.held_for += self.dt
        elif not self.reached:
            self.target += LIFT_SPEED * self.dt
        if self.held_for >= LIFT_HOLD_S or self.time >= LIFT_TIMEOUT_S + LIFT_HOLD_S:
            self.done = True
            return None
        velocity = np.asarray(clip_velocity, dtype=float)
        up = (m.CLIP_MASS + m.STACK_MASS) * 9.81 + LIFT_KP * (self.target - clip["origin"][2]) - LIFT_KD * velocity[2]
        return np.array((-LIFT_SIDE_KD * velocity[0], -LIFT_SIDE_KD * velocity[1], np.clip(up, 0., LIFT_MAX_N)))

    def held(self, poses, clip):
        """Per plate: carried by the clip through the check."""
        if not self.reached or self.held_for < LIFT_HOLD_S - 1e-9:
            return [False] * len(poses)
        moved = np.linalg.norm(under_lips(poses, clip) - self.before, axis=1)
        return [bool(a and b and d <= LIFT_SLIP_M) for a, b, d in zip(self.was_in, in_jaws(poses, clip), moved)]


def measure(poses, clip):
    """Everything about the end state that does not depend on the grippers or the lift check."""
    long_side, short_side = m.misalignment(poses)
    placed, along, off = clip_placed(poses, clip)
    inside = in_jaws(poses, clip)
    return {"long_edges_mm": round(1000 * long_side, 2), "short_edges_mm": round(1000 * short_side, 2),
            "sheets_in_jaws": m.PER_PLATE * sum(inside), "clip_placed": placed,
            "clip_from_edge_middle_mm": round(1000 * along, 1), "clip_angle_deg": round(off, 1),
            "lip_gap_mm": round(1000 * clip["lip_gap"], 2), "largest_angle_deg": round(largest_angle(poses), 1)}


def score(state, *, let_go, held_sheets, zeroed):
    """``state``: :func:`measure`. ``held_sheets``: sheets carried through the lift check, or None if none was run."""
    if zeroed:
        return 0.
    flush = max(state["long_edges_mm"], state["short_edges_mm"]) / 1000.
    held = state["sheets_in_jaws"] if held_sheets is None else held_sheets
    if let_go and held_sheets == SHEETS and flush <= FLUSH_M and state["clip_placed"]:
        return 1.
    if let_go and held_sheets == SHEETS and flush <= ROUGH_M:
        return .6
    return .3 if held >= HALF else 0.


@dataclass
class PaperGrade:
    """Running grade of one trial. Call :meth:`update` once per control step. The lift check is run by the
    adapter when ``wants_lift`` is set (once each time the full-score state has been let go for 2 s, and at the
    trial's last step) and reported with :meth:`lifted`."""
    hold: int = 0
    zeroed: str = ""
    held_sheets: int | None = None
    lift_tick: int | None = None
    lift_spent: bool = False
    tick: int = 0
    log: dict = field(default_factory=dict)

    def update(self, poses, clip, tcp, touching, *, on_paper=None, last=False):
        """``poses``: every plate's (position, rotation matrix), bottom plate first. ``clip``:
        ``paper_model.clip_state``. ``tcp``: both gripper centre points. ``touching``: whether any part of the
        robot touches a sheet or the clip; ``on_paper``: whether it touches a sheet (default: the same).
        Tabletop frame. ``last``: this is the trial's final step."""
        self.tick += 1
        flat = np.concatenate([np.ravel(p) for p, _ in poses] + [np.ravel(r) for _, r in poses] + [np.ravel(tcp), clip["origin"]])
        if not np.isfinite(flat).all():
            self.hold = 0
            return self._grade(0., {"error": "nonfinite state"}, False)
        if not self.zeroed and fell(poses):
            self.zeroed = "a sheet left the table"
        state = measure(poses, clip)
        gap = clearance(tcp, poses, clip)
        clear = bool(gap >= CLEAR_M and not touching)
        self.hold = self.hold + 1 if clear else 0
        let_go = self.hold >= HOLD_TICKS
        if self.lift_tick is not None and self.lift_tick != self.tick:
            self.held_sheets, self.lift_tick = None, None          # the scene has gone on since the lift check
        full = (let_go and state["sheets_in_jaws"] == SHEETS and state["clip_placed"] and not self.zeroed
                and max(state["long_edges_mm"], state["short_edges_mm"]) <= 1000 * FLUSH_M)
        if not clear:
            self.lift_spent = False                         # touched again: a later let-go earns a new lift check
        wants_lift = bool(let_go and not self.zeroed and state["sheets_in_jaws"] >= HALF and ((full and not self.lift_spent) or last))
        value = score(state, let_go=let_go, held_sheets=self.held_sheets, zeroed=self.zeroed)
        return self._grade(value, dict(state, hands_clear=clear, gripper_clearance_mm=round(1000 * gap, 1), let_go=let_go),
                           wants_lift)

    def lifted(self, grade, held_plates):
        """Fold the lift check's result (per plate: carried) into ``grade`` (the update of the same step)."""
        self.held_sheets = m.PER_PLATE * int(sum(held_plates))
        self.lift_tick = self.tick
        self.lift_spent = True
        grade = dict(grade, held_sheets=self.held_sheets, lift_checked=True, wants_lift=False)
        grade["score"] = score(grade, let_go=grade["let_go"], held_sheets=self.held_sheets, zeroed=self.zeroed)
        return grade

    def _grade(self, value, detail, wants_lift):
        return {"score": value, "hold_ticks": self.hold, "zeroed": self.zeroed, "held_sheets": self.held_sheets,
                "lift_checked": False, "wants_lift": wants_lift, **detail}


@dataclass(frozen=True)
class PaperScorer:
    name: str = "paper_jog_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["paper_grade"]
        if grade.get("zeroed"):
            text = "0: " + grade["zeroed"]
        elif "error" in grade:
            text = grade["error"]
        else:
            text = (f"{grade['sheets_in_jaws']} of {SHEETS} sheets in the clip's jaws, "
                    f"{grade['held_sheets'] if grade['lift_checked'] else 'no lift check:'} carried when lifted by the clip; "
                    f"edges out of flush by {grade['long_edges_mm']} mm (long sides) and {grade['short_edges_mm']} mm (short sides); "
                    f"clip {'on' if grade['clip_placed'] else 'not on'} the middle of a short edge; "
                    f"{'let go' if grade['let_go'] else 'not let go'}")
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return PaperScorer()
