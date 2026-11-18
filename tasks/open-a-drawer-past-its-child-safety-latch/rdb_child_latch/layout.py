"""Every dimension of the storage drawer and its child-safety latch, in metres. NumPy only.

Tabletop frame: x forward from the line through the two arm bases, y to the robot's left, z up from the
table top. The drawer slides along x; "open" is the distance from the frame's front face to the outer face
of the drawer front. The drawer's own frame has its origin on the outer face of its front, on the table plane,
x pointing into the unit.

Each number is marked PUBLISHED (read on the maker's or a retailer's page, see README.md), DERIVED from
published numbers, or ESTIMATE.
"""

import numpy as np

INCH = .0254

# -- the storage drawer: Sterilite 16 Qt. Stacking Drawer, model 2301 -------------------------------------
FRAME_SIZE = (.432, .365, .175)          # PUBLISHED 17 x 14 3/8 x 6 7/8 in: front to back, width, height
DRAWER_INNER = (.416, .302, .124)        # PUBLISHED "interior at bottom of drawer" 16 3/8 x 11 7/8 x 4 7/8 in
FRAME_FRONT_X = .390                     # where the unit stands: its front face, centred on y = 0
RAIL_Z = .152                            # ESTIMATE underside of the frame's top at the opening
FRAME_WALL = .008                        # modelled side wall (the real moulding is about 3 mm; outside sizes are true)
FRAME_BACK = .002
FLOOR_Z = .022                           # ESTIMATE inside surface of the drawer's floor above the table
RIM_Z = FLOOR_Z + DRAWER_INNER[2]        # DERIVED top of the drawer's side walls: 146 mm
DRAWER_FLOOR_T = .008
DRAWER_WALL = .008                       # modelled (real about 2.5 mm; the inside sizes are true)
DRAWER_BACK = .003
FRONT_T = .003                           # ESTIMATE drawer front panel thickness (a moulding about 3 mm thick)
FRONT_WIDTH = .343                       # ESTIMATE: fills the frame's opening with 3 mm to spare each side
FRONT_Z = (.012, .148)                   # ESTIMATE: bottom and top edge of the front panel
PULL_SIZE = (.012, .120, .008)           # ESTIMATE the moulded pull: a lip standing 12 mm proud of the front
PULL_Z = .132                            # its middle height
OPEN_START = 0.                          # the drawer starts shut
OPEN_LIMIT = .300                        # ESTIMATE the built-in stop that keeps the drawer from coming right out
RUNNER_FRICTION_N = 3.                   # ESTIMATE force to slide the empty drawer on its plastic runners
DRAWER_MASS = .50                        # ESTIMATE (maker's shipping weight for a case of six: 19 lb, about 1.4 kg each)

# -- the latch: Safety 1st Spring-Loaded Cabinet & Drawer Latch (48392 family) ----------------------------
LATCH_PUBLISHED = (2.5 * INCH, 1. * INCH, 1.5 * INCH)   # PUBLISHED retailer size: 2.5 long x 1 high x 1.5 wide in
CATCH_OPEN = 1.5 * INCH                  # PUBLISHED "When drawer is opened 1.5 in the latch will engage catch"
LATCH_Y = 0.                             # PUBLISHED catch "on upper frame at center of drawer space"
LATCH_LENGTH = LATCH_PUBLISHED[0]        # from the drawer front's inner face to the back of the hook
LATCH_WIDTH = LATCH_PUBLISHED[2]
PLATE_T = .003                           # ESTIMATE the latch's base, screwed to the inside of the drawer front
HINGE_X = FRONT_T + PLATE_T              # drawer frame: the arm pivots on a pin at the front of its base
ARM_TOP_Z = .138                         # ESTIMATE top of the latch arm when it rests up against its stop
ARM_T = .005                             # ESTIMATE arm thickness (modelled solid; the real arm is a ribbed moulding)
HINGE_Z = ARM_TOP_Z - ARM_T / 2
HOOK_RISE = .009                         # ESTIMATE the hook stands this far above the arm
HOOK_LONG = .005                         # ESTIMATE the hook's length along the arm
HOOK_FACE_X = FRONT_T + LATCH_LENGTH - HOOK_LONG          # drawer frame: the hook's catching (front) face
HOOK_TOP_Z = ARM_TOP_Z + HOOK_RISE       # 147 mm
PLATE_Z = (HOOK_TOP_Z - LATCH_PUBLISHED[1], HOOK_TOP_Z)   # the latch is 1 in high overall
RAMP_HALF = HOOK_RISE / np.sqrt(2.)      # the hook's rounded back: a square turned on edge, centred on the arm's top at
                                         # the hook's back end, so its upper back face is a 45-degree ramp
LATCH_MASS = .012                        # ESTIMATE
# The spring: a torsion spring at the pin holding the arm up against its stop.
LATCH_STIFFNESS = .30                    # ESTIMATE N m/rad
LATCH_PRELOAD = .20                      # ESTIMATE rad: the spring is wound this far past the stop
LATCH_TRAVEL = np.radians(10.)           # ESTIMATE the arm can be pressed down this far before it bottoms out
LATCH_DAMPING = .003                     # ESTIMATE N m s/rad

# -- the catch: screwed to the underside of the frame's top -------------------------------------------------
CATCH_FACE_X = FRAME_FRONT_X - CATCH_OPEN + HOOK_FACE_X   # DERIVED tabletop x of the catch's back face: the hook
CATCH_DEPTH = .010                       # ESTIMATE                                           meets it at 1.5 in
CATCH_DROP = .010                        # ESTIMATE the catch hangs this far below the frame's top
CATCH_WIDTH = .025                       # ESTIMATE
CATCH_LOW_Z = RAIL_Z - CATCH_DROP        # 142 mm
ENGAGE = HOOK_TOP_Z - CATCH_LOW_Z        # DERIVED how far the hook overlaps the catch: 5 mm

# -- friction (all ESTIMATE) --------------------------------------------------------------------------------
FRICTION = {"frame": .20, "drawer": .20, "latch": .30,      # polypropylene and the latch's plastic
            "pull": .60}                                # the pull's lip where the rubber-padded jaws hold it

ACTORS = ("drawer", "latch")
MASSES = (DRAWER_MASS, LATCH_MASS)


def drawer_x(open_m):
    """Tabletop x of the drawer body's origin (outer face of its front, on the table plane)."""
    return FRAME_FRONT_X - open_m


def hinge_point(open_m):
    """Tabletop position of the latch's pivot."""
    return np.array((drawer_x(open_m) + HINGE_X, LATCH_Y, HINGE_Z))


def hook_drop(angle):
    """How far the hook's catching corner has gone down when the arm is pressed ``angle`` rad below its stop."""
    dx, dz = HOOK_FACE_X - HINGE_X, HOOK_TOP_Z - HINGE_Z
    return dx * np.sin(angle) + dz * (1. - np.cos(angle))


def _release_angle():
    low, high = 0., float(LATCH_TRAVEL)
    for _ in range(60):
        mid = (low + high) / 2
        low, high = (mid, high) if hook_drop(mid) < ENGAGE else (low, mid)
    return high


RELEASE_ANGLE = _release_angle()         # DERIVED the arm pressed this far (about 5 degrees) frees the hook


def spring_torque(angle):
    """Torque (N m) the spring exerts when the arm is pressed ``angle`` rad down from its stop."""
    return LATCH_STIFFNESS * (LATCH_PRELOAD + angle)


def press_force(angle, at_x):
    """Force (N) a fingertip pressing straight down ``at_x`` (drawer frame) needs to hold the arm at ``angle``."""
    return spring_torque(angle) / max(at_x - HINGE_X, 1e-6)


def catch_open():
    """Opening at which the hook's front face meets the catch (the arm at rest): 1.5 in."""
    return FRAME_FRONT_X + HOOK_FACE_X - CATCH_FACE_X


def hook_clear_open():
    """Opening beyond which the part of the hook that reaches above the catch's underside is in front of it."""
    back = FRONT_T + LATCH_LENGTH + HOOK_RISE - (CATCH_LOW_Z - ARM_TOP_Z)     # the ramp's x at that height
    return FRAME_FRONT_X + back - (CATCH_FACE_X - CATCH_DEPTH)


def unit_box(open_m):
    """Bounding box (low, high) of the frame, the drawer and its pull, for the hands-clear rule."""
    front = drawer_x(max(0., open_m)) - PULL_SIZE[0]
    low = np.array((front, -FRAME_SIZE[1] / 2, 0.))
    high = np.array((FRAME_FRONT_X + FRAME_SIZE[0], FRAME_SIZE[1] / 2, FRAME_SIZE[2]))
    return low, high


def start_poses():
    """Tabletop start position (x, y, z) of every body's origin. Drawer: outer face of its front on the table
    plane. Latch: its pivot."""
    return {"drawer": (drawer_x(OPEN_START), 0., 0.), "latch": tuple(hinge_point(OPEN_START))}


assert abs(catch_open() - CATCH_OPEN) < 1e-9
assert abs(ENGAGE - .005) < 1e-9 and HOOK_TOP_Z < RAIL_Z - .004
assert RELEASE_ANGLE < LATCH_TRAVEL - np.radians(3.)
