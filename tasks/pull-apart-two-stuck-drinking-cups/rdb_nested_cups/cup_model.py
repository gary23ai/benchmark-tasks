"""Cup dimensions, masses, start poses and the stuck fit. Pure Python and NumPy: no simulator imports.

Every length is in metres and every pose is in tabletop coordinates: x forward from the line through
the two arm bases, y to the robot's left, z up from the table top.

Product: IKEA KALAS tumbler, article 004.613.79 (six-pack of polypropylene children's tumblers).
Published by the retailer: rim diameter 8 cm, volume 23 cl (8 oz), polypropylene, six cups stacked are
16.5 to 17 cm tall and the pack weighs 0.19 kg (7 oz).
From a third-party drawing site, not confirmed by the retailer: one cup is 8.1 cm tall and weighs 30 g.
NOT published anywhere we found, and therefore estimates to be confirmed with calipers on a bought cup:
base diameter (66 mm) and wall thickness (1.5 mm). They were chosen so that the model reproduces the
published stack pitch ((170 - 81) / 5 = 17.8 mm; the model gives 17.4 mm) and the 30 g mass.
"""

import math

import numpy as np

TABLE_TOP_WORLD_Z = 0.7474999999999999  # station.TABLE_TOP; fixed_scene.py asserts they agree

# -- one cup -----------------------------------------------------------------------------
HEIGHT = 0.081
RIM_RADIUS = 0.040  # outside, at the rim (published 8 cm diameter)
BASE_RADIUS = 0.033  # outside, at the base (estimate, see above)
WALL = 0.0015  # wall and base thickness (estimate, see above)
MASS = 0.030
SLOPE = (RIM_RADIUS - BASE_RADIUS) / HEIGHT  # outside radius gained per metre of height
HALF_ANGLE = math.atan(SLOPE)  # 4.94 degrees
SEGMENTS = 24  # flat wall panels per cup; radii above are the panels' distances from the axis

# -- friction (estimates) ----------------------------------------------------------------
# Polypropylene on polypropylene, dry. The cup material uses the "min" combine mode, so this is also the
# ceiling for finger pad on cup and cup on table.
FRICTION = 0.35

# -- the stuck fit -------------------------------------------------------------------------
# Two identical tapered cups touch wall to wall when the inner one sits FREE_PITCH above the outer one. On
# real cups, pressing the inner one further in makes the walls bear on each other and friction holds the
# pair (the taper is self-locking: tan(4.94 deg) = 0.086 is below the friction coefficient).
#
# That friction lock cannot be reproduced with this simulator's contacts. Seven runs with the walls pressed
# together on compliant contact springs showed the inner cup creeping out untouched until only its weight
# held it: whatever the overlap, the push-out force settled at 0.63 to 0.72 N against a weight of 0.29 N,
# which means the contact friction that actually acted was about 0.05, not the 0.35 set on the material,
# and the "hold" left over was close to nothing. So the fit is a rule instead: a bond between the two cups
# that breaks for good when the force between them passes FIT_BREAK_FORCE_N (or the moment between them
# FIT_BREAK_TORQUE_NM). Until then the pair is one body; afterwards nothing joins the cups.
FREE_PITCH = WALL / math.sin(HALF_ANGLE)
FIT_CLEARANCE = 0.0005  # the bonded inner cup sits this far above FREE_PITCH, so the rigid walls do not touch
SEATED_PITCH = FREE_PITCH + FIT_CLEARANCE
FIT_BREAK_FORCE_N = 6.0  # about 20 times one cup's weight
FIT_BREAK_TORQUE_NM = 0.3  # a 6 N pull at the rim (34 mm off axis) is 0.2 N m, so a straight pull breaks by force
FIT_FRAME_HEIGHT = 0.049  # the bond acts at this height above the outer cup's base, mid-way up the nested walls
# The push that seats a real pair to the same pull-apart force: with wall force N, pull F = N (mu cos a - sin a)
# and press P = N (sin a + mu cos a).
PRESS_IN_N = FIT_BREAK_FORCE_N * ((math.sin(HALF_ANGLE) + FRICTION * math.cos(HALF_ANGLE))
                                  / (FRICTION * math.cos(HALF_ANGLE) - math.sin(HALF_ANGLE)))

# -- collision-only simplification of the inner cup's rim -----------------------------------
# The top of the inner cup, the band that stands above the outer cup and is the only part a hand can reach,
# collides as rigid blocks whose outside faces are the true surface and which are BAND_THICKNESS deep inward
# (the drawn wall stays 1.5 mm). Nothing has to enter the inner cup further than a finger, and the band never
# touches the outer cup.
# Why so thick: the station's gripper is a position drive of 5,000 N/m per finger whose target cannot go
# below "fingers touching", so pinching a wall of thickness t squeezes with at most 5,000 N/m x t/2: 3.75 N
# on the true 1.5 mm wall, 30 N on 12 mm. A real gripper sets force, not position (about 40 N per finger).
# The band's thickness stands in for that; a pinch on it was measured holding steadily in a probe run
# (17.7 N per finger on an 8 mm band).
BAND_START = 0.067  # height above the inner cup's base where the rigid band begins
BAND_THICKNESS = 0.012
# Below the band the inner cup is solid for collision (stacked rigid cylinders just inside its wall): nothing
# has to go deeper into it than a fingertip (13 mm), and a plain solid is what this simulator's gripper holds
# reliably when it squeezes the pair from outside.
PLUG_CLEARANCE = 0.0003  # between the solid and the wall's inside surface
PLUG_TOP = BAND_START - 0.001
PLUG_LAYERS = 4

# -- start poses (base centres, both cups upright, same axis) -----------------------------
STACK_XY = (0.275, 0.035)
REST_GAP = 0.0002  # the outer cup is authored this far above the table and settles onto it
OUTER_START = (*STACK_XY, REST_GAP)
INNER_START = (*STACK_XY, REST_GAP + SEATED_PITCH)
CUPS = ("outer", "inner")
START = np.array((OUTER_START, INNER_START))


def outside_radius(z):
    """Outside radius of a cup at height ``z`` above its base."""
    return BASE_RADIUS + SLOPE * z


# -- diagnostic probe (one simulator job; set PROBE = False for the task itself) ---------------
# With PROBE on, the scene gains two plain rigid cylinders, the "scripted" policy becomes a sequence of slow
# test closes (probe.py) and every step logs finger joints, link poses and contact vectors. The score of a
# probe run means nothing.
PROBE = False
PROBE_MAX_STEPS = 3200
PROBE_BODIES = {  # name: (half extents x, y, z, or None for a cylinder; radius; height; mass; base centre)
    "probe80": (None, 0.040, 0.081, 0.050, (0.275, 0.185, 0.0002)),  # rigid cylinder as wide as the cup's rim
    "probe66": (None, 0.033, 0.081, 0.050, (0.275, 0.300, 0.0002)),  # rigid cylinder as wide as the cup's base
    "probebox": ((0.015, 0.040, 0.0405), None, 0.081, 0.050, (0.180, 0.185, 0.0002)),  # rigid box, 80 mm across y
}
