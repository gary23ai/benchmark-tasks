"""Every dimension, mass, spring and friction value of the task, in SI units. NumPy only.

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the
table top. Each number is marked PUBLISHED (a standard or a product page, see TECHNICAL.md), DERIVED from
published numbers, or ESTIMATE.

Sheet frame (every plate): origin at the plate's middle, +u along the short side (215.9 mm), +v along the
long side (279.4 mm), +w through the thickness.

Clip frame: origin on the hinge line in the middle of the clip's back, +u from the back toward the jaws'
lips (the way the clip is pushed onto paper), +n across the paper's thickness, +w along the clip's width.
"""

import os

import numpy as np

INCH = .0254

# -- the paper -------------------------------------------------------------------------------------------------
SHEET_U, SHEET_V = 8.5 * INCH, 11. * INCH    # PUBLISHED US Letter, 215.9 x 279.4 mm
N_SHEETS = 36
SHEET_T = .0001                              # PUBLISHED 20 lb bond copy paper is about 0.097 to 0.104 mm thick
GSM = .075                                   # PUBLISHED 20 lb bond = 75 g/m2
SHEET_MASS = SHEET_U * SHEET_V * GSM         # DERIVED 4.52 g
K = int(os.environ.get("PAPER_JOG_K", 9))    # rigid plates the stack is modelled as
PER_PLATE = N_SHEETS // K                    # sheets a plate stands for
PLATE_T = PER_PLATE * SHEET_T                # 0.4 mm
PLATE_MASS = PER_PLATE * SHEET_MASS          # 18.1 g
STACK_T = K * PLATE_T                        # 3.6 mm
STACK_MASS = K * PLATE_MASS                  # 163 g
PAPER_FRICTION = (.30, .25)                  # ESTIMATE static, dynamic, paper on paper
PLATES = tuple(f"sheet_{i}" for i in range(K))

# Where each plate starts, bottom plate first: shift along the short side, shift along the long side (m) and
# turn about the vertical (degrees). A fixed table, not random. Neighbours differ by 5 to 8 mm along the
# short side and 9 to 13 mm along the long side; every plate is turned 1 to 2.5 degrees.
OFFSETS = ((-.003, .006, 2.0), (.004, -.006, -1.5), (-.004, .007, 1.0), (.003, -.005, -2.5), (-.002, .004, 1.5),
           (.004, -.007, -1.0), (-.003, .006, 2.5), (.003, -.004, -2.0), (-.004, .005, 1.0))[:K]

# -- what the stack starts on ------------------------------------------------------------------------------------
# "reams": two wrapped 500-sheet reams of US Letter copy paper, one on the other, lying with their long side
# pointing away from the robot (each 8.5 x 11 x 2 in). "block": the MDF riser of the earlier versions.
SUPPORT = os.environ.get("PAPER_JOG_SUPPORT", "reams")
REAM = (11 * INCH, 8.5 * INCH, .051)         # one ream as it lies: along x, along y, height
BLOCK = {"reams": (REAM[0], REAM[1], 2 * REAM[2]), "block": (.160, .200, .080)}[SUPPORT]     # along x, along y, height
BLOCK_NEAR_X = .570                          # its face toward the robot
BLOCK_Y = -.100                              # its middle
OVERHANG = .050                              # the stack's near long edge stands this far out over that face
STACK_X = BLOCK_NEAR_X - OVERHANG + SHEET_U / 2      # middle of the stack at the start
STACK_Y = BLOCK_Y
START_GAP = .00005                           # plates are spawned this far apart and settle

# -- the jogging area -------------------------------------------------------------------------------------------
JOG_X = .310                                 # the stack is stood on its long edge in the plane x = JOG_X

# -- the binder clip: medium, 1-1/4 in (32 mm) wide ---------------------------------------------------------------
CLIP_W = 1.25 * INCH                         # PUBLISHED width, 31.75 mm
CLIP_CAPACITY = .625 * INCH                  # PUBLISHED 5/8 in
CLIP_MASS = .009                             # ESTIMATE
JAW_L = .021                                 # ESTIMATE back to lips
BACK_HALF = .008                             # ESTIMATE half the width of the clip's back
JAW_T = .0012                                # modelled sheet-metal thickness (real about 0.4 mm)
LIP_R = .0012                                # the rolled lip, modelled as a capsule along the width
HANDLE_L = .028                              # ESTIMATE the wire handle reaches this far behind the back
HANDLE_W = .022                              # ESTIMATE width of the wire loop
HANDLE_T = .0016                             # PUBLISHED-like wire diameter, estimate
OPEN_HALF = np.radians(17.2)                 # each half turns this far when the handles are squeezed together
HANDLE_SPLAY = OPEN_HALF                     # at rest each handle splays out by this; squeezed, they are parallel
CLIP_OPEN = 2 * OPEN_HALF                    # hinge travel
CLIP_RATE = .25                              # ESTIMATE N m/rad of the spring between the two halves
CLIP_PRELOAD = .126                          # ESTIMATE N m with the jaws shut: 6 N at the lips
CLIP_DAMPING = .002
CLIP_FRICTION = (.35, .30)                   # ESTIMATE painted steel on paper
JAW_GAP_OPEN = 2 * JAW_L * np.sin(OPEN_HALF)             # DERIVED 12.4 mm between the lips, handles squeezed

# -- the clip's stand: an upright fin the clip is parked on, handles up -------------------------------------------
FIN = (.040, .003, .035)                     # along x, thickness, height; acrylic, fixed
FIN_XY = (.250, .300)

# -- solver settings of the loose bodies (not physical quantities) ------------------------------------------------
PHYSICS_HZ = 240
SOLVER_ITERATIONS = (int(os.environ.get("PAPER_JOG_POS_ITERS", 64)), 8)      # position, velocity
CONTACT_OFFSET = float(os.environ.get("PAPER_JOG_CONTACT_OFFSET", .0005))    # m, plates and clip
MAX_DEPENETRATION = float(os.environ.get("PAPER_JOG_DEPEN", .5))             # m/s

# -- other friction (ESTIMATE) ---------------------------------------------------------------------------------
FRICTION = {"paper": PAPER_FRICTION, "clip": CLIP_FRICTION, "block": (.40, .35), "acrylic": (.30, .25)}
ROBOT_FRICTION = .5                          # the stock arm and table have PhysX's default material, combined by average


# -- where the clip starts: parked on the fin, jaws down, handles up ------------------------------------------------
PARK_HALF = float(np.arcsin(FIN[1] / 2 / JAW_L))        # each half is turned this far by the fin between the lips
PARK_ROT = np.array(((0., 1., 0.), (1., 0., 0.), (0., 0., -1.)))     # columns: where the clip's n, w and u axes point
PARK_POS = (FIN_XY[0], FIN_XY[1], FIN[2] + .0005)
CLIP_HALVES = ("clip_a", "clip_b")
ACTORS = (*PLATES, *CLIP_HALVES)
MASSES = (*([PLATE_MASS] * K), CLIP_MASS / 2, CLIP_MASS / 2)


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))


def clip_start():
    """Tabletop start pose (position, rotation matrix) of the clip's two halves."""
    return {"clip_a": (np.array(PARK_POS), PARK_ROT @ rot_y(PARK_HALF)),
            "clip_b": (np.array(PARK_POS), PARK_ROT @ rot_y(-PARK_HALF))}


def clip_state(pose_a, pose_b):
    """The clip as a whole from its two halves' (position, rotation matrix): origin on the hinge line, rotation
    (columns: n across the paper, w along the width, u toward the lips), opening angle (rad) and the gap between
    the lips (m)."""
    (pa, ra), (pb, rb) = pose_a, pose_b
    rel = np.asarray(rb, dtype=float).T @ np.asarray(ra, dtype=float)
    opening = float(np.arctan2(rel[0, 2], rel[0, 0]))
    rotation = np.asarray(rb, dtype=float) @ rot_y(opening / 2)
    return {"origin": (np.asarray(pa, dtype=float) + np.asarray(pb, dtype=float)) / 2, "rotation": rotation,
            "opening": opening, "lip_gap": 2 * JAW_L * float(np.sin(max(opening, 0.) / 2))}


def clamp_force(thickness):
    """Force (N) the lips press with on something ``thickness`` thick."""
    angle = 2 * np.arcsin(thickness / 2 / JAW_L)
    return (CLIP_PRELOAD + CLIP_RATE * angle) / JAW_L


def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))


def quat_xyzw(rotation):
    """Rotation matrix to (x, y, z, w)."""
    r = np.asarray(rotation, dtype=float)
    w = np.sqrt(max(0., 1. + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return ((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(0., 1. + r[i, i] - r[j, j] - r[k, k])) * 2
    q = [0., 0., 0., (r[k, j] - r[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    return tuple(q)


def start_poses():
    """Tabletop start pose (position, rotation matrix) of every plate: flat on the block, long side along y,
    shifted and turned by OFFSETS."""
    base = np.array(((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)))      # u along x (short side), v along y (long side)
    poses = {}
    for i, (du, dv, yaw) in enumerate(OFFSETS):
        z = BLOCK[2] + PLATE_T / 2 + i * (PLATE_T + START_GAP) + START_GAP
        poses[PLATES[i]] = (np.array((STACK_X + du, STACK_Y + dv, z)), rot_z(np.radians(yaw)) @ base)
    return poses


def corners(position, rotation):
    """The four corners of a plate's mid-plane, tabletop frame: (-u,-v), (+u,-v), (+u,+v), (-u,+v)."""
    half = np.array(((-1, -1), (1, -1), (1, 1), (-1, 1))) * (SHEET_U / 2, SHEET_V / 2)
    r = np.asarray(rotation, dtype=float)
    return np.asarray(position, dtype=float) + half[:, :1] * r[:, 0] + half[:, 1:] * r[:, 1]


def misalignment(poses):
    """How far the plates' edges are from flush, measured as a person would with a steel rule on the squared
    stack laid flat: for each of the four sides, the distance between the plate edge that sticks out most and
    the one that sits back most, taken at both ends of that side. Returns (worst over the two long sides,
    worst over the two short sides), in metres. ``poses``: [(position, rotation matrix)] of every plate.

    The frame is the stack's own: the average of the plates' in-plane axes, so the result does not depend on
    where the stack lies or how it is turned."""
    rotations = np.array([np.asarray(r, dtype=float) for _, r in poses])
    u = rotations[:, :, 0].mean(axis=0)
    u /= np.linalg.norm(u)
    w = np.cross(u, rotations[:, :, 1].mean(axis=0))
    w /= np.linalg.norm(w)
    v = np.cross(w, u)
    pts = np.array([corners(p, r) for p, r in poses])            # plate, corner, xyz
    pu, pv = pts @ u, pts @ v                                     # plate, corner
    # long sides run along v at u = -half and u = +half: corners (0, 3) and (1, 2); compare their u positions
    long_side = max(np.ptp(pu[:, c]) for c in range(4))
    short_side = max(np.ptp(pv[:, c]) for c in range(4))
    return float(long_side), float(short_side)


assert K * PER_PLATE == N_SHEETS and len(OFFSETS) == K
assert JAW_GAP_OPEN > STACK_T + .002
assert STACK_X + SHEET_U / 2 <= BLOCK_NEAR_X + BLOCK[0] + .010
