"""Score a trial from the poses of the two halves and the grippers' positions. NumPy only.

Every rule can be judged from the trial video, photos of the glass and a ruler:

- on the glass: a half's pad lies flat against a face of the fixed pane;
- paired: the two halves are on the glass on opposite faces and their outlines line up through it, centres
  within 15 mm (which half is on which face does not matter);
- let go: no part of the robot touches either half and each gripper's centre point (midway between its
  fingertips) is at least 8 cm from the middle of each half's handle bar;
- cleaned area: the marker band is 150 x 30 mm; the scorer divides it into 180 squares of 5 mm, and a square
  counts once the pad of the half on the band's face (the outdoor face) has slid over its centre while the
  halves were paired and no part of the robot was touching that half;
- out of bounds: a gripper's centre point crosses the plane of the wall anywhere but through the open light
  (over the top of the wall, or round its ends).

Score: 0 if a gripper went out of bounds, or if the halves were never paired; 0.25 if they were paired at some
moment; 0.5 if at the end they have been paired, let go and still for 2 s; 0.75 if in addition at least half
of the band's area was cleaned; 1.0 if at least 90 % of it was.
"""

from dataclasses import dataclass, field

import numpy as np

from . import cleaner_model as model

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
FLAT_COS = float(np.cos(np.radians(5.)))
PAD_GAP_RANGE = (-.0015, .002)  # pad face to glass face: from 1.5 mm of contact give to a 2 mm gap
PAIR_OFFSET = .015  # between the two halves' centres, measured along the glass
CLEAR_OF_GRIPPER = .080  # gripper centre point to the middle of a half's handle bar
STILL = .010  # m/s
SLIDING = .005  # m/s; a pad that only rests on the band does not clean it
HALF_AREA = .50  # fraction of the band's area for 0.75
FULL_AREA = .90  # fraction of the band's area for 1.0
CELL_MM2 = (1000. * model.CELL) ** 2
# Centres of the band's squares on the outdoor face, in pane coordinates: rows from the bottom, columns along u.
_U, _Z = np.meshgrid(model.CELL_U, model.CELL_Z)
GRID_POINTS = np.column_stack((_U.ravel(), np.full(_U.size, model.FACE_Y["outside"]), _Z.ravel()))
GRID_SHAPE = _U.shape  # (rows, columns) = (6, 30)


def on_glass(position, rot):
    """+1 or -1 for the face (indoor, n > 0, or outdoor, n < 0) that the half's pad lies flat on; 0 if
    neither. ``position`` and ``rot`` are in pane coordinates."""
    side = 1. if position[1] >= 0. else -1.
    flat = side * rot[1, 2] >= FLAT_COS  # the half's back points straight away from the pane
    gap = side * position[1] - model.PANE_T / 2
    within = model.PANE_X[0] <= position[0] <= model.PANE_X[1] and \
        model.PANE_CLEAR_Z[0] <= position[2] <= model.PANE_CLEAR_Z[1]
    return int(side) if flat and PAD_GAP_RANGE[0] <= gap <= PAD_GAP_RANGE[1] and within else 0


def in_opening(point):
    """True when a point in pane coordinates lies within the open light's rectangle (seen square on)."""
    return bool(model.OPENING_U[0] <= point[0] <= model.OPENING_U[1] and model.OPENING_Z[0] <= point[2] <= model.OPENING_Z[1])


@dataclass
class WindowGrade:
    """Running grade of one trial. Call :meth:`update` once per control step."""
    hold: int = 0
    ever_paired: bool = False
    violation: str | None = None
    rubbed: np.ndarray = field(default_factory=lambda: np.zeros(GRID_POINTS.shape[0], dtype=bool))
    last_tcp: np.ndarray | None = None

    def update(self, pose, velocity, tcp, touching):
        """``pose``: per half (inside, outside), frame origin XYZ in tabletop coordinates then an XYZW
        quaternion in the tabletop frame. ``velocity``: per half, linear velocity. ``tcp``: both gripper
        centre points. ``touching``: per half, whether any part of the robot touches it."""
        pose, velocity, tcp = (np.asarray(v, dtype=float) for v in (pose, velocity, tcp))
        if not (np.isfinite(pose).all() and np.isfinite(velocity).all() and np.isfinite(tcp).all()):
            self.hold = 0
            return self._grade(False, False, np.zeros(2, dtype=int), float("nan"))
        # Everything below is in pane coordinates: u along the wall, n square to it (n > 0 indoors), z up.
        pose = pose.copy()
        pose[:, :3] = model.to_pane(pose[:, :3])
        tcp = model.to_pane(tcp)
        # Out of bounds: a gripper centre point that crosses the wall's mid-plane must do so inside the open light.
        if self.last_tcp is not None:
            for before, now in zip(self.last_tcp, tcp):
                a, b = before[1] - model.WALL_MID_N, now[1] - model.WALL_MID_N
                if a * b < 0. or (a == 0. and b != 0.):
                    crossing = before + (now - before) * (a / (a - b))
                    if not in_opening(crossing):
                        self.violation = self.violation or "a gripper crossed the wall's plane outside the open light"
        self.last_tcp = tcp.copy()
        rots = [model.PANE_ROT.T @ model.rotation(row[3:7]) for row in pose]
        seated = np.array([on_glass(pose[i, :3], rots[i]) for i in range(2)])
        offset = float(np.linalg.norm((pose[0, :3] - pose[1, :3])[[0, 2]]))
        paired = bool(seated[0] * seated[1] == -1 and offset <= PAIR_OFFSET)
        self.ever_paired |= paired
        speed = np.linalg.norm(velocity[:, :3], axis=1)
        grips = [pose[i, :3] + rots[i] @ model.GRIP_POINT for i in range(2)]
        clear = min(float(np.linalg.norm(p - g)) for p in tcp for g in grips) >= CLEAR_OF_GRIPPER
        hands_free = bool(clear and not any(touching))
        wiper = int(np.argmin(seated))  # the half on the outdoor face, where the band is
        if paired and not touching[wiper] and speed[wiper] >= SLIDING:
            local = (GRID_POINTS - pose[wiper, :3]) @ rots[wiper]  # the squares' centres in that half's frame
            self.rubbed |= model.in_pad(local[:, :2])
        self.hold = self.hold + 1 if paired and hands_free and speed.max() <= STILL else 0
        return self._grade(paired, hands_free, seated, offset)

    def _grade(self, paired, hands_free, seated, offset):
        fraction = float(self.rubbed.mean())
        if self.violation or not self.ever_paired:
            score = 0.
        elif self.hold < HOLD_TICKS:
            score = .25
        else:
            score = 1. if fraction >= FULL_AREA else .75 if fraction >= HALF_AREA else .5
        return {"score": score, "paired": paired, "hands_free": hands_free, "ever_paired": bool(self.ever_paired),
                "violation": self.violation, "on_glass": [int(s) for s in seated],
                "pair_offset_mm": round(1000. * offset, 1), "hold_ticks": self.hold,
                "area_cleaned_fraction": round(fraction, 4),
                "area_cleaned_mm2": round(CELL_MM2 * int(self.rubbed.sum()), 1),
                "cells_cleaned": [bool(r) for r in self.rubbed]}


@dataclass(frozen=True)
class WindowWallScorer:
    name: str = "window_wall_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["magnet_grade"]
        if grade["violation"]:
            text = f'0: {grade["violation"]}'
        elif not grade["ever_paired"]:
            text = "0: the two halves never held each other through the glass"
        elif grade["hold_ticks"] < HOLD_TICKS:
            text = "the halves were paired, but at the end they are not paired, let go and still"
        else:
            text = f'cleaner left holding on the glass; {100 * grade["area_cleaned_fraction"]:.0f} % of the band cleaned'
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return WindowWallScorer()
