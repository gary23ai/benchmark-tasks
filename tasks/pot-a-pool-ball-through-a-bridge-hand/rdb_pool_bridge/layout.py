"""Geometry of the pool table, the balls, the cue and the shot. NumPy only, so the scorer can be tested alone.

Frame (the "tabletop frame" of the station): x forward from the line through the two arm bases, y to the
robot's left, z up from the top of the robot's bench (0.76 m above the floor). Units: metres, radians.

Sources (README): WPA Recommended Equipment Specifications (bed height, cushion nose height, rail width, pocket
cut angles and shelves, ball size and mass, cue limits); Dr Dave Alciatore's physical-properties page (friction,
rolling resistance, restitution); retailer listings of 58-inch, 19 oz, 13 mm house cues; balance-point guides.
Anything marked "estimate" was not found on a page.
"""

import math

import numpy as np

IN = 0.0254
DEG = math.pi / 180.

# -- balls (WPA: 2 1/4 in +- 0.005, 5 1/2 to 6 oz) --------------------------------------------------------------
BALL_R = 2.25 * IN / 2                      # 0.028575
BALL_MASS = 0.163                           # 5.75 oz, middle of 156-170 g
BALLS = ("cue", "one", "three")             # white cue ball, yellow 1-ball (the named ball), red 3-ball
BALL_COLOURS = {"cue": (0.93, 0.92, 0.86), "one": (0.95, 0.70, 0.05), "three": (0.75, 0.06, 0.05)}

# -- table: a 7-ft home table, playing surface 78 x 39 in ----------------------------------------------------------
PLAY_L = 78 * IN                            # along y (the robot stands at a long rail)
PLAY_W = 39 * IN                            # along x
BED_Z = 0.027                               # cloth surface: 31 in above the floor (WPA bed height 29 1/4 to 31 in)
FLOOR_Z = -0.76                             # the station floor in this frame
SLATE_T = 1. * IN                           # WPA: slate at least 1 in
NOSE_H = 0.635 * 2 * BALL_R                 # WPA: cushion nose 63.5 % of the ball's diameter above the bed (36.3 mm)
RAIL_H = 0.048                              # rail cap above the bed (estimate)
RAIL_TOP = BED_Z + RAIL_H
RAIL_W = 5. * IN                            # nose of cushion to outer edge (WPA 4 to 7 1/2 in)
CUSHION_W = 2. * IN                         # cloth-covered cushion, featherstrip to nose (WPA 1 7/8 to 2 1/8 in)
CORNER_MOUTH = 5.0 * IN                     # home tables (estimate); WPA tournament 4 1/2 to 4 5/8 in
SIDE_MOUTH = 5.5 * IN                       # 1/2 in wider than the corners, as WPA notes
CORNER_CUT = 142. * DEG                     # WPA horizontal pocket cut angle, corners
SIDE_CUT = 104. * DEG                       # and sides
CORNER_SHELF = 1.5 * IN                     # WPA corner shelf 1 to 2 1/4 in
SIDE_SHELF = .25 * IN                       # WPA side shelf 0 to 3/8 in
CORNER_HOLE_R = .058                        # radius of the slate's pocket hole (estimate)
SIDE_HOLE_R = .055
POCKET_DEPTH = .10                          # drop pocket floor below the cloth (estimate)
X_N = .200                                  # nose of the near long cushion (the rail's outer edge is at x = 0.073)
X_F = X_N + PLAY_W

# -- the gripper's closed loop used as a bridge (from the YAM collision hulls, see README) ------------------------
PAD_TIP_PAST_TCP = .013
LOOP_CEILING = .0744                        # underside of the finger carriages, above the fingertips
LOOP_GAP = .008                             # commanded pad gap: the 13 mm shaft cannot pass the pads below it
LOOP_WALL_HALF = .0075                      # half-width between the fingers at the shaft's height with that gap
PRESS_TIPS = .0015                          # fingertips on the cloth (nominal)

# -- cue: two-piece 58 in house cue, 19 oz, 13 mm tip --------------------------------------------------------------
CUE_L = 58 * IN                             # 1.4732
CUE_MASS = .539                             # 19 oz
CUE_CM_S = CUE_L - 18.5 * IN                # balance point 18 1/2 in from the butt (guides: 17 to 20 in)
JOINT_S = 29 * IN                           # 29 in shaft
# (s from the tip, diameter) breakpoints; linear in between
CUE_PROFILE = ((0., .013), (.33, .013), (JOINT_S, .021), (JOINT_S, .0215), (JOINT_S + .0254, .0215),
               (JOINT_S + .0254, .022), (1.0, .0255), (1.27, .0275), (1.46, .0295), (CUE_L, .0295))
TIP_L = .012                                # leather tip (s 0 .. 0.012), domed: 11 mm radius ("dime" shape)
TIP_DOME_R = .011
FERRULE_S = (.012, .032)
CUE_SEGMENTS = (  # name, s0, s1, material
    ("tip", 0., TIP_L, "tip_leather"), ("ferrule", FERRULE_S[0], FERRULE_S[1], "ferrule"),
    ("shaft", FERRULE_S[1], .33, "shaft_maple"), ("taper", .33, JOINT_S, "shaft_maple"),
    ("joint", JOINT_S, JOINT_S + .0254, "joint_ring"), ("forearm", JOINT_S + .0254, 1.0, "forearm"),
    ("wrap", 1.0, 1.27, "wrap"), ("sleeve", 1.27, 1.46, "sleeve"), ("bumper", 1.46, CUE_L, "bumper"))


def cue_diameter(s):
    s = float(np.clip(s, 0., CUE_L))
    for (s0, d0), (s1, d1) in zip(CUE_PROFILE, CUE_PROFILE[1:]):
        if s0 <= s <= s1 and s1 > s0:
            return d0 + (d1 - d0) * (s - s0) / (s1 - s0)
    return CUE_PROFILE[-1][1]


# -- the shot -------------------------------------------------------------------------------------------------------
PITCH = 7. * DEG                            # cue elevation at address
CUT = 30. * DEG                             # cut angle of the 1-ball toward the near rail
D1 = .12                                    # cue ball centre to the ghost-ball position
D2 = .20                                    # 1-ball centre to the aim point in the pocket's mouth
AIM_OFFSET = .006                           # aim point moved from the mouth's centre toward the near-rail jaw
ADDRESS_GAP = .015                          # tip behind the cue ball's surface at address
GRIP_S = .80                                # where the right hand holds the cue, from the tip
BRIDGE_Y = .040                             # where the left hand bridges (y), chosen for reach
CUE_DIR_XY = np.array((0., 1.))             # the shot is played along +y
OB_DIR_XY = np.array((-math.sin(CUT), math.cos(CUT)))


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, -s, 0.), (s, c, 0.), (0., 0., 1.)))


BALL_Z = BED_Z + BALL_R
BRIDGE_AXIS_Z = BED_Z + PRESS_TIPS + LOOP_CEILING - cue_diameter(.29) / 2     # cue axis in the loop at address
D_BRIDGE = (BRIDGE_AXIS_Z - BALL_Z) / math.sin(PITCH)                        # ball centre to bridge, along the cue
BRIDGE_S = D_BRIDGE - BALL_R - ADDRESS_GAP                                   # bridge contact from the tip at address
CORNER_J = CORNER_MOUTH / math.sqrt(2.)                                      # jaw to the nose lines' corner


def _shot():
    """Place the table so that the shot fits the arms: the bridge at (x, BRIDGE_Y), cue along +y."""
    jaw_dir = unit((-1., -1.))                         # from the mouth's centre toward the near-rail jaw
    # mouth centre M = (X_N + j/2, Y_L - j/2); aim A = M + offset * jaw_dir; 1-ball O = A - D2 w;
    # ghost G = O - 2R w; cue ball C = G - D1 u; bridge B = C - D_BRIDGE cos(PITCH) u
    a_x = X_N + CORNER_J / 2 + AIM_OFFSET * jaw_dir[0]
    o_x = a_x - D2 * OB_DIR_XY[0]
    g_x = o_x - 2 * BALL_R * OB_DIR_XY[0]
    back = D1 + D_BRIDGE * math.cos(PITCH)
    y_l = (BRIDGE_Y + back + 2 * BALL_R * OB_DIR_XY[1] + D2 * OB_DIR_XY[1] - AIM_OFFSET * jaw_dir[1] + CORNER_J / 2)
    m = np.array((X_N + CORNER_J / 2, y_l - CORNER_J / 2))
    a = m + AIM_OFFSET * jaw_dir
    o = a - D2 * OB_DIR_XY
    g = o - 2 * BALL_R * OB_DIR_XY
    c = g - D1 * CUE_DIR_XY
    assert abs(c[0] - g_x) < 1e-12
    return y_l, a, o, g, c


Y_L, AIM, ONE_XY, GHOST_XY, CUE_BALL_XY = _shot()
Y_R = Y_L - PLAY_L
Y_SIDE = Y_L - PLAY_L / 2
BRIDGE_XY = CUE_BALL_XY - D_BRIDGE * math.cos(PITCH) * CUE_DIR_XY
THREE_XY = np.array((.84, -.42))           # out of play, behind the shot (estimate of a typical leave)
BALL_START = {"cue": np.r_[CUE_BALL_XY, BALL_Z], "one": np.r_[ONE_XY, BALL_Z], "three": np.r_[THREE_XY, BALL_Z]}

# the cue at address: axis direction (from the butt toward the tip) and the tip's position
CUE_AXIS = np.array((0., math.cos(PITCH), -math.sin(PITCH)))
CUE_BALL = BALL_START["cue"]
TIP_ADDRESS = CUE_BALL - (BALL_R + ADDRESS_GAP) * CUE_AXIS
GRIP_ADDRESS = TIP_ADDRESS - GRIP_S * CUE_AXIS
BRIDGE_ADDRESS = TIP_ADDRESS - BRIDGE_S * CUE_AXIS               # cue axis point inside the loop


def cue_point(tip, axis, s):
    """Point on the cue's axis ``s`` behind the tip."""
    return np.asarray(tip, dtype=float) - s * np.asarray(axis, dtype=float)


# -- the cue's start: lying on the near rail's cap, tip toward the marked pocket -----------------------------------
RAIL_CAP_X = (X_N - RAIL_W, X_N - CUSHION_W)                   # wooden cap between the outer edge and the cushion
CUE_REST_X = sum(RAIL_CAP_X) / 2
CUE_REST_TIP_Y = .40
_r_tip, _r_butt = cue_diameter(0.02) / 2, cue_diameter(CUE_L) / 2
CUE_REST_PITCH = -math.atan2(_r_butt - _r_tip, CUE_L)        # resting on its butt and its ferrule: tip a little down
CUE_REST_AXIS = np.array((0., math.cos(CUE_REST_PITCH), math.sin(CUE_REST_PITCH)))
CUE_REST_TIP = np.array((CUE_REST_X, CUE_REST_TIP_Y, RAIL_TOP + _r_tip + .0005))


def cue_rotation(axis):
    """Rotation of the cue body (local +x along the axis toward the tip, local z as near up as possible)."""
    x = unit(axis)
    y = unit(np.cross((0., 0., 1.), x))
    return np.column_stack((x, y, np.cross(x, y)))


# -- pockets --------------------------------------------------------------------------------------------------------
def _corner(sx, sy):
    vx = X_N if sx < 0 else X_F
    vy = Y_L if sy > 0 else Y_R
    j = CORNER_J
    turn = math.pi - CORNER_CUT                                  # 38 degrees
    jaws = (np.array((vx, vy - sy * j)), np.array((vx - sx * j, vy)))
    facings = (np.array((sx * math.sin(turn), sy * math.cos(turn))), np.array((sx * math.cos(turn), sy * math.sin(turn))))
    mouth = (jaws[0] + jaws[1]) / 2
    out = unit((sx, sy))
    hole = mouth + (CORNER_SHELF + CORNER_HOLE_R) * out
    return dict(kind="corner", jaws=jaws, facings=facings, mouth=mouth, out=out, hole=hole, hole_r=CORNER_HOLE_R,
                width=CORNER_MOUTH)


def _side(sx):
    vx = X_N if sx < 0 else X_F
    turn = math.pi - SIDE_CUT                                    # 76 degrees from the nose line
    jaws = (np.array((vx, Y_SIDE + SIDE_MOUTH / 2)), np.array((vx, Y_SIDE - SIDE_MOUTH / 2)))
    facings = (np.array((sx * math.sin(turn), -math.cos(turn))), np.array((sx * math.sin(turn), math.cos(turn))))
    mouth = (jaws[0] + jaws[1]) / 2
    out = np.array((float(sx), 0.))
    hole = mouth + (SIDE_SHELF + SIDE_HOLE_R) * out
    return dict(kind="side", jaws=jaws, facings=facings, mouth=mouth, out=out, hole=hole, hole_r=SIDE_HOLE_R,
                width=SIDE_MOUTH)


POCKETS = {"near_left": _corner(-1, 1), "near_side": _side(-1), "near_right": _corner(-1, -1),
           "far_left": _corner(1, 1), "far_side": _side(1), "far_right": _corner(1, -1)}
TARGET_POCKET = "near_left"
OUTER = (X_N - RAIL_W, X_F + RAIL_W, Y_R - RAIL_W, Y_L + RAIL_W)       # the table's outline (rail outer edges)
CHALK_XYZ = np.array((X_N - RAIL_W + .030, Y_L - CORNER_J - .040, RAIL_TOP + .011))   # chalk cube by the marked pocket


def in_pocket(position):
    """Name of the pocket a ball has dropped into (centre below the cloth by more than a radius, over a pocket
    hole or its drop pocket), else None."""
    p = np.asarray(position, dtype=float)
    if p[2] > BED_Z - BALL_R:
        return None
    for name, pocket in POCKETS.items():
        if np.linalg.norm(p[:2] - pocket["hole"]) <= pocket["hole_r"] + .015:
            return name
    return None


def off_table(position):
    """A ball that is neither on the table nor in a pocket: outside the rails' outline or fallen below the bed."""
    p = np.asarray(position, dtype=float)
    if in_pocket(p):
        return False
    x0, x1, y0, y1 = OUTER
    return bool(not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1) or p[2] < BED_Z - .05)


def past_mouth(position, pocket=TARGET_POCKET):
    """The ball's centre has crossed the line between the pocket's two jaw points (it is in the jaws)."""
    pk = POCKETS[pocket]
    return bool((np.asarray(position, dtype=float)[:2] - pk["mouth"]) @ pk["out"] > 0.)


def on_cloth(position):
    p = np.asarray(position, dtype=float)
    return bool(abs(p[2] - BALL_Z) < .004 and X_N <= p[0] <= X_F and Y_R <= p[1] <= Y_L)
