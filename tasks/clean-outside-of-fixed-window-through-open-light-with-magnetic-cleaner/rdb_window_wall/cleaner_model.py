"""The wall with its window, the two halves of the magnetic cleaner and the marker band. Pure Python + NumPy.

Two frames, both in metres with z up from the table top.

Tabletop (world) coordinates: x forward from the line through the two arm bases, y to the robot's left.

Pane coordinates (u, n, z): u along the wall (the same as tabletop y), n square to it and positive INDOORS,
towards the robot, measured from the middle of the glass. The robot is indoors; n < 0 is outdoors. Everything
about the window (wall, opening, glass, band, where the halves start, the scorer's rules, the demonstration's
waypoints) is written in pane coordinates; `to_world` and `to_pane` convert.

Each half has its own frame: origin at the centre of the triangular pad face, +Z out of the back (towards the
handle), +X along the handle bar and the pad's long edge, +Y towards the pad's apex.

Numbers marked ESTIMATE are not published for the product and must be measured on a bought one (README).
"""

import numpy as np

TABLE_TOP_WORLD_Z = 0.7474999999999999  # yam_arena.station.TABLE_TOP; checked in fixed_scene.py

# -- wall and window -----------------------------------------------------------------------------
WALL_X = 0.400  # tabletop x of the wall's indoor face
WALL_T = 0.012  # 12 mm plywood
WALL_U = (-0.640, 0.640)  # the wall runs right across the 1300 mm table
WALL_H = 0.560
FOOT_SIZE = (0.060, 0.300, 0.040)  # two feet under the wall's ends (along u, across n, tall), clamped to the table
FOOT_U = (-0.610, 0.610)
OPENING_U = (-0.380, 0.050)  # the open light: 430 wide ...
OPENING_Z = (0.130, 0.450)  # ... and 320 tall, above a 130 mm sill
STILE_U = (0.050, 0.060)  # the post between the open light and the fixed light
PANE_T = 0.004  # 4 mm single glazing, inside the cleaner's 3-8 mm rating
PANE_X = (STILE_U[1], 0.610)  # the fixed light's glass along u: 550 wide
PANE_Z = OPENING_Z  # 320 tall
PANE_CLEAR_Z = PANE_Z
PANE_MASS = 1.76  # 550 x 320 x 4 mm of glass at 2500 kg/m3 (the pane is fixed in the simulation)
FACE_Y = {"inside": PANE_T / 2, "outside": -PANE_T / 2}  # n coordinate of each face of the glass
# The glass is bedded flush with the wall's outdoor face, so its indoor face is 8 mm back from the wall's.
GLASS_MID_X = WALL_X + WALL_T - PANE_T / 2  # tabletop x of the middle of the glass
WALL_N = (-PANE_T / 2, WALL_T - PANE_T / 2)  # the wall's two faces in n: outdoor, indoor
WALL_MID_N = (WALL_N[0] + WALL_N[1]) / 2
PANE_YAW_DEG = 90.  # u is tabletop +y, n is tabletop -x
PANE_PIVOT_U = 0.
PANE_PIVOT_XY = (GLASS_MID_X, 0.000)
_YAW = np.radians(PANE_YAW_DEG)
PANE_ROT = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))  # columns: u, n, z in the tabletop frame
PANE_QUAT = (0., 0., float(np.sin(_YAW / 2)), float(np.cos(_YAW / 2)))  # XYZW
_PIVOT_WORLD = np.array((*PANE_PIVOT_XY, 0.))
_PIVOT_PANE = np.array((PANE_PIVOT_U, 0., 0.))


def to_world(points):
    """Pane coordinates (u, n, z) to tabletop coordinates (x, y, z); one point or an array of points."""
    return (np.asarray(points, dtype=float) - _PIVOT_PANE) @ PANE_ROT.T + _PIVOT_WORLD


def to_pane(points):
    """Tabletop coordinates to pane coordinates; one point or an array of points."""
    return (np.asarray(points, dtype=float) - _PIVOT_WORLD) @ PANE_ROT + _PIVOT_PANE
SIDE = {"inside": 1., "outside": -1.}  # which side of y = 0 the half belongs on
HALVES = ("inside", "outside")

# -- one half of the cleaner -------------------------------------------------------------------
PAD_BASE = 0.150  # retailer: 15 cm
PAD_HEIGHT = 0.130  # retailer: 13 cm
HALF_THICK = 0.045  # retailer: 4.5 cm, taken as one half including its handle (ESTIMATE of what the figure covers)
# Triangle corners in the half's frame (centroid at the origin).
TRIANGLE = np.array(((-PAD_BASE / 2, -PAD_HEIGHT / 3), (PAD_BASE / 2, -PAD_HEIGHT / 3), (0., 2 * PAD_HEIGHT / 3)))
PAD_T = 0.004  # microfibre pad (ESTIMATE)
BODY_T = 0.022  # pad plus plastic shell (ESTIMATE)
BAR_SIZE = (0.080, 0.024, 0.010)  # handle bar: length, width pinched by the gripper, thickness (ESTIMATE)
BAR_Z = (BODY_T + 0.013, HALF_THICK)  # the bar's underside is 13 mm above the shell
POST_SIZE = (0.012, 0.024, BAR_Z[0] - BODY_T)
POST_X = 0.034
GRIP_POINT = np.array((0., 0., (BAR_Z[0] + BAR_Z[1]) / 2))  # middle of the handle bar
MASS = 0.200  # kg per half (ESTIMATE; a comparable two-piece cleaner is listed at 440 g for the pair)
PART_MASS = {"pad": 0.010, "shell": 0.160, "bar": 0.020, "post": 0.005}  # sums to MASS with two posts
PAD_FRICTION = 0.25  # dry microfibre on glass (ESTIMATE)
SHELL_FRICTION = 0.40
HANDLE_FRICTION = 0.80
GLASS_FRICTION = 0.40
WALL_FRICTION = 0.50

# -- magnets -----------------------------------------------------------------------------------
# Two block magnets per half, magnetised through their thickness, side by side along the handle bar. Both
# magnets of the inside half show a north pole at the pad and both of the outside half a south pole, so the
# two pads attract in any in-plane orientation and a pad repels the other half's back. (ESTIMATE of the
# layout: a comparable product is described as having two rare-earth magnets per piece; whether real pairs
# are keyed to one in-plane orientation is not published.)
MAGNET_X = 0.035  # magnets sit at x = +-35 mm
MAGNET_DEPTH = 0.005  # pad face to the magnet's near face
MAGNET_T = 0.010
POLE_SIGN = {"inside": 1., "outside": -1.}  # sign of the pole nearer the pad
SOFTENING = 0.006  # m; stands in for the finite size of the pole faces
FORCE_CAP = 25.  # N per pole pair
FIELD_RANGE = 0.30  # m between half centres; beyond this the force is under 0.01 N and is skipped
MU0 = 4e-7 * np.pi
# No holding force is published for this class of cleaner. The model is sized from the product's rating:
# on the thickest rated glass (8 mm) the pads must still pull together hard enough that friction carries a
# released half with a margin, 10 N for a 0.2 kg half at friction 0.25 (2 N of weight against 2.5 N).
RATED_GLASS = 0.008
HOLD_AT_RATED_N = 10.  # ESTIMATE (design value)


def poles(half):
    """((position in the half's frame, sign), ...) of the four point poles of one half."""
    out = []
    for x in (-MAGNET_X, MAGNET_X):
        out.append((np.array((x, 0., MAGNET_DEPTH)), POLE_SIGN[half]))
        out.append((np.array((x, 0., MAGNET_DEPTH + MAGNET_T)), -POLE_SIGN[half]))
    return out


def _pair_force(offset, strength):
    """Force on the outside half (N) when its frame origin is displaced by ``offset`` = (dx, gap, dz) from
    the mirror position of the inside half: ``gap`` is the distance between the two pad faces."""
    dx, gap, dz = offset
    total = np.zeros(3)
    for p1, s1 in poles("inside"):
        a = np.array((p1[0], p1[2], p1[1]))  # inside half on the glass: its +Z is world +y
        for p2, s2 in poles("outside"):
            b = np.array((p2[0] + dx, -gap - p2[2], p2[1] + dz))
            r = b - a
            total += MU0 * strength ** 2 * s1 * s2 / (4 * np.pi) * r / (r @ r + SOFTENING ** 2) ** 1.5
    return total


POLE_STRENGTH = float(np.sqrt(HOLD_AT_RATED_N / _pair_force((0., RATED_GLASS, 0.), 1.)[1]))  # A m


def hold_force(gap=PANE_T, dx=0., dz=0.):
    """(pull towards the glass, pull back towards alignment along x, along z) on the outside half, N."""
    f = _pair_force((dx, gap, dz), POLE_STRENGTH)
    return float(f[1]), float(-np.sign(dx) * f[0]) if dx else 0., float(-np.sign(dz) * f[2]) if dz else 0.


# -- layout --------------------------------------------------------------------------------------
WORK_Z = 0.310  # height of the pad centres while the cleaner is on the glass in the demonstration
# Start: both halves lie indoors, pad down on the table, handle up, handle bar parallel to the wall. The blue
# (inside) half lies in front of the fixed light with its apex towards the wall; the white (outside) half
# lies in front of the stile with its apex towards the robot, where the right arm can lay its hand along the
# handle bar. They are 300 mm apart, beyond the magnets' range. START_UN is in pane coordinates (u, n).
START_UN = {"inside": (0.330, 0.150), "outside": (0.030, 0.130)}
START_SETTLE = 0.0003  # authored this far above the table; the scene settles for 0.5 s
# XYZW quaternions in the pane frame. Inside half: turned half a turn about vertical, apex towards -n.
START_QUAT_PANE = {"inside": (0., 0., 1., 0.), "outside": (0., 0., 0., 1.)}

# The marker band: on the OUTDOOR face of the glass, horizontal, 150 mm long and 30 mm tall. BAND_U is its
# extent along u; it starts 350 mm from the stile, beyond what a hand-held half can reach (README, "Reach").
# It lies in the lower, wider part of the pad's triangle when the pad's centre is at WORK_Z, and high enough on
# the glass for the overhead camera to see it over the left arm at its ready pose. The scorer divides it into
# 5 mm squares.
BAND_U = (0.410, 0.560)
BAND_Z = (WORK_Z - 0.030, WORK_Z)
CELL = 0.005
CELL_U = np.arange(BAND_U[0] + CELL / 2, BAND_U[1], CELL)  # 30 columns
CELL_Z = np.arange(BAND_Z[0] + CELL / 2, BAND_Z[1], CELL)  # 6 rows
CELLS = len(CELL_U) * len(CELL_Z)


def rotation(quat_xyzw):
    x, y, z, w = quat_xyzw
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def quat_multiply(a, b):
    """XYZW quaternion of rotation ``a`` applied after rotation ``b``."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


# World start poses.
START_QUAT = {half: quat_multiply(PANE_QUAT, START_QUAT_PANE[half]) for half in HALVES}
START = {half: tuple(float(v) for v in to_world((*START_UN[half], START_SETTLE))) for half in HALVES}
START_XY = {half: START[half][:2] for half in HALVES}


def on_glass_rotation(half):
    """Rotation, in the pane frame, of a half sitting on its face of the pane with its apex up and its
    handle bar level. Multiply by PANE_ROT for the tabletop frame."""
    back = np.array((0., SIDE[half], 0.))
    up = np.array((0., 0., 1.))
    return np.column_stack((np.cross(up, back), up, back))


def in_pad(points_xy):
    """Which points, given in a half's frame (x, y), lie on its triangular pad."""
    points_xy = np.atleast_2d(points_xy)
    inside = np.ones(len(points_xy), dtype=bool)
    for i in range(3):
        a, b = TRIANGLE[i], TRIANGLE[(i + 1) % 3]
        edge, rel = b - a, points_xy - a
        inside &= edge[0] * rel[:, 1] - edge[1] * rel[:, 0] >= 0.
    return inside


def centre_of_mass():
    """Centre of mass of one half in its own frame (uniform parts)."""
    z = (PART_MASS["pad"] * PAD_T / 2 + PART_MASS["shell"] * (PAD_T + BODY_T) / 2
         + PART_MASS["bar"] * (BAR_Z[0] + BAR_Z[1]) / 2 + 2 * PART_MASS["post"] * (BODY_T + BAR_Z[0]) / 2)
    return np.array((0., 0., z / MASS))


COM = centre_of_mass()
