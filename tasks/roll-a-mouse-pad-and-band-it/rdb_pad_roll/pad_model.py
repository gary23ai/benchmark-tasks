"""Every dimension, mass, spring and friction of the mouse pad, the carton it lies on, the paper band and its cradle.

NumPy only. Metres, kilograms, newtons. Tabletop frame: x forward from the line through the two arm bases, y to
the robot's left, z up from the table top.

Status of each number: PUBLISHED (a catalogue size), DERIVED (follows from other numbers), MEASURED (read off
this simulation, see results/pad-physics.json) or ESTIMATE (to be measured on the bought item; README lists them).
"""

import numpy as np

# --- mouse pad: cloth-topped natural-rubber foam, 250 x 210 x 3 mm ----------------------------------------
PAD_L, PAD_W, PAD_T = .250, .210, .003           # PUBLISHED: the common "medium" office size
PAD_MASS = .105                                  # ESTIMATE: 90-120 g for this size
STRIPS = 25                                      # rigid strips across the pad, hinged to each other along its length
STRIP_W = PAD_L / STRIPS                         # 10 mm
# A strip's collider is narrower than the strip, so that neighbours folded to the hinge stop do not touch:
# 2 * (PAD_T / 2) * tan(50 deg / 2) = 1.4 mm.
COLLIDER_W = .0085
LINK_MASS = 2e-4                                 # the massless-in-effect link between a hinge and its shear slide
STRIP_MASS = (PAD_MASS - (STRIPS - 1) * LINK_MASS) / STRIPS
PAD_EI = 1.0e-3                                  # N m^2 ESTIMATE: bending stiffness of the 210 mm wide pad
HINGE_K = PAD_EI / STRIP_W                       # DERIVED: 0.10 N m/rad; one hinge stands for 10 mm of pad
HINGE_DAMPING = .010                             # N m s/rad ESTIMATE: the foam's slow, damped spring-back
HINGE_ARMATURE = 5e-6
HINGE_LIMIT_DEG = 50.                            # the tightest curl: an inside diameter of about 18 mm
# In-plane slide between neighbouring strips along the hinge line. A real pad does not shear; when it is rolled
# crooked it winds as a helix and each turn is offset along the roll. The slide stands in for that offset.
SHEAR_K = 150.                                   # N/m
SHEAR_DAMPING = 2.                               # N s/m
SHEAR_LIMIT = .0015                              # +-1.5 mm a hinge
SOLVER_ITERATIONS = (16, 4)                      # position, velocity: PhysX TGS, for the pad and the band
STRIP_INERTIA = (1.5e-5, 3e-7, 1.5e-5)           # about the hinge axis (y) raised from 3.6e-8 kg m^2 for the solver
FRICTION_RUBBER = 1.0                            # ESTIMATE: underside (natural rubber foam)
FRICTION_CLOTH = .5                              # ESTIMATE: top (polyester knit)

# --- carton the pad lies on: a closed corrugated mailer box, fixed to the table ----------------------------
CARTON_L, CARTON_W, CARTON_H = .280, .220, .120  # along x, along y, height
FRICTION_CARTON = .8                             # ESTIMATE
PAD_X0 = .245                                    # the pad's near short edge
PAD_Y0 = .040                                    # the pad's right long edge
OVERHANG = .025                                  # the near short edge sticks out over the carton's near edge
SIDE_OVERHANG = .010                             # the right long edge sticks out over the carton's right edge
CARTON_X0 = PAD_X0 + OVERHANG
CARTON_Y0 = PAD_Y0 + SIDE_OVERHANG
PAD_Z = CARTON_H + PAD_T / 2                     # mid-plane of the flat pad
PAD_Y = PAD_Y0 + PAD_W / 2                       # its centre line

# --- band: a ring of heavy kraft paperboard (a length cut from a mailing tube), open at both ends -----------
# It is what keeps the rolled pad rolled, and its bore is the gauge of how tightly the pad was rolled.
BAND_ID = .070                                   # DERIVED: the roll the demonstration makes (61 to 63 across, held) plus 8
BAND_WALL = .003
BAND_L = .080                                    # along the roll
BAND_MASS = .039                                 # DERIVED: 0.7 g/cm^3 spiral-wound kraft board
BAND_STAVES = 16                                 # the bore is a 16-sided prism of flat boxes; BAND_ID is its inscribed circle
BAND_OD = BAND_ID + 2 * BAND_WALL
FRICTION_BAND = .5                               # ESTIMATE: kraft paper

# --- cradle: two V saddles fixed to the table, the band lies in them along x --------------------------------
CRADLE_X, CRADLE_Y = .340, -.360                 # under the middle of the band
SADDLE_X = (-.020, .020)                         # the two saddles, from the middle of the band
SADDLE_T = .012                                  # thickness of a saddle along the band
SADDLE_SLOPE_DEG = 35.                           # each face of the V, from level
SADDLE_FACE = .045                               # length of a face up the slope
SADDLE_PLATE = .008
BAND_REST_Z = .050                               # height of the band's axis when it lies in the cradle
FRICTION_CRADLE = .5

BAND_START = np.array((CRADLE_X, CRADLE_Y, BAND_REST_Z))


def strip_centres_flat(x0=PAD_X0, y=PAD_Y, z=PAD_Z):
    """Centres of the strips of a flat pad whose near edge is at x0 (strip 0 is nearest the robot)."""
    return np.array([(x0 + (i + .5) * STRIP_W, y, z) for i in range(STRIPS)])


PAD_START = strip_centres_flat()
ROOT = STRIPS // 2                               # the articulation's root link is the middle strip

# --- the roll, as plain geometry (used by the demonstration, the dry run and the tests) ---------------------
CORE_R = STRIP_W / (2 * np.tan(np.radians(HINGE_LIMIT_DEG) / 2))        # mid-plane radius of the tightest first turn


def wound_length(turn, core=CORE_R, pitch=PAD_T):
    """Length of pad taken up after the first strip has turned by ``turn`` radians (an Archimedean spiral)."""
    return core * turn + pitch * turn ** 2 / (4 * np.pi)


def turn_for_length(length, core=CORE_R, pitch=PAD_T):
    a, b = pitch / (4 * np.pi), core
    return (-b + np.sqrt(b * b + 4 * a * length)) / (2 * a)


def roll_radius(turn, core=CORE_R, pitch=PAD_T):
    """Mid-plane radius of the turn being laid down at ``turn``."""
    return core + pitch * turn / (2 * np.pi)


def band_staves(inner=None, wall=BAND_WALL, n=BAND_STAVES):
    """(angle about the axis, centre radius, half width) of each flat stave of the band wall."""
    inner = BAND_ID / 2 if inner is None else inner
    half = (inner + wall) * np.tan(np.pi / n) + .0002
    return [(2 * np.pi * k / n, inner + wall / 2, half) for k in range(n)]


def saddle_faces():
    """The two sloping plates of a saddle as (centre y, centre z, roll about x) relative to the band's axis: each
    plate's upper face is tangent to the band's outer circle."""
    s = np.radians(SADDLE_SLOPE_DEG)
    out = []
    for sign in (1., -1.):
        normal = np.array((sign * np.sin(s), np.cos(s)))            # (y, z): from the plate up toward the axis
        centre = -normal * (BAND_OD / 2 + SADDLE_PLATE / 2)
        out.append((float(centre[0]), float(centre[1]), float(-sign * s)))
    return out


def tight_bends(limit_deg=HINGE_LIMIT_DEG, pitch=PAD_T):
    """Fold angle (rad) of every hinge 1..STRIPS-1 in the tightest roll wound from strip 0, the first two strips
    held flat together (hinge 1 stays straight). Each further hinge folds as far as the turn under it allows."""
    core = STRIP_W / (2 * np.tan(np.radians(limit_deg) / 2))
    bends, turned = [0.], 0.
    for _ in range(2, STRIPS):
        radius = core + pitch * turned / (2 * np.pi)
        bend = min(np.radians(limit_deg), 2 * np.arctan(STRIP_W / (2 * radius)))
        bends.append(bend)
        turned += bend
    return np.array(bends)


def winding(turn, limit_deg=HINGE_LIMIT_DEG, pitch=PAD_T):
    """The pad partly rolled: strips 0 and 1 turned by ``turn`` (rad) about the roll axis, the hinges behind them
    folded one after another to their tight angles, the rest of the pad still flat and where it started.

    Returns (hinge points, fold angles): hinge points is (STRIPS + 1, 2), the x (from the pad's near edge) and z
    (above the flat pad's mid-plane) of the pad's near edge, hinge 1 .. hinge STRIPS-1 and its far edge. Past the
    last hinge the finished roll turns as one piece about its middle, on the spot.
    """
    full = tight_bends(limit_deg, pitch)
    total = float(full.sum())
    bends, left = np.zeros(STRIPS - 1), min(turn, total)
    for j in range(1, STRIPS - 1):               # hinge index j + 1 in 1-based numbering; hinge 1 never folds
        bends[j] = min(full[j], left)
        left -= bends[j]
    points = np.zeros((STRIPS + 1, 2))
    points[:, 0] = np.arange(STRIPS + 1) * STRIP_W
    angle = 0.
    for j in range(STRIPS - 1, 0, -1):           # walk from the far, flat end back to the near edge
        angle += bends[j - 1]
        step = STRIP_W * np.array((-np.cos(angle), np.sin(angle)))
        points[j - 1] = points[j] + step
    if turn > total:                             # all wound: turn the whole roll about its middle
        extra = turn - total
        centre = roll_middle(points)
        c, s = np.cos(extra), np.sin(extra)
        rel = points - centre
        points = centre + np.column_stack((c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]))
    return points, bends


def roll_middle(points):
    """Middle of a finished roll: the centre of the smallest upright box round its hinge points (x, z)."""
    return (points.min(axis=0) + points.max(axis=0)) / 2


def full_turn(limit_deg=HINGE_LIMIT_DEG, pitch=PAD_T):
    """Turn (rad) of the first strips when the last hinge has just folded: the whole pad is on the roll."""
    return float(tight_bends(limit_deg, pitch).sum())


def pinch_pose(turn, limit_deg=HINGE_LIMIT_DEG, pitch=PAD_T):
    """Where the middle of strips 0 and 1 (hinge 1, what the gripper holds) is at ``turn``: x from the pad's near
    edge, z above the flat pad's mid-plane."""
    points, _ = winding(turn, limit_deg, pitch)
    return points[1]


def rolled_length(turn, limit_deg=HINGE_LIMIT_DEG, pitch=PAD_T):
    """Length of pad already on the roll at ``turn``: one strip for every hinge folded home, and the two held flat."""
    full = tight_bends(limit_deg, pitch)
    left, strips = min(turn, float(full.sum())), 0.
    for bend in full[1:]:
        strips += min(bend, left) / bend
        left = max(0., left - bend)
    return strips * STRIP_W


def tight_diameter(strips_on):
    """About how far across a tightly rolled part of the pad is, with ``strips_on`` strips on the roll (MEASURED in
    this simulation, results/pad-physics.json: 30 mm for the first curl of six strips, 44 mm for the whole pad)."""
    return .026 + .00075 * strips_on
