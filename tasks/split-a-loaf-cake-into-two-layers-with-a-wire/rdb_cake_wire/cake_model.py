"""Sizes, masses, start poses and material numbers of the loaf cake, its stand and the cutting wire.

Pure Python and NumPy. Lengths in metres, masses in kilograms, forces in newtons. Positions are tabletop
coordinates: x forward from the line through the two arm bases, y to the robot's left, z up from the table top.
Every number marked ESTIMATE is not published by a maker or retailer; see the README for how to check it.
"""

import numpy as np

TABLE_TOP_WORLD_Z = 0.7475            # station table top above the simulator's floor

# --- work surface: matt black self-adhesive vinyl film stuck to the table top ---------------------------
# 0.1 mm thick, so it is colour only: it has no collider and things rest on the table through it.
FILM_X = (.060, .600)
FILM_Y = (-.300, .300)
FILM_DRAWN_THICKNESS = .0005                   # drawn half a millimetre proud of the table so that it renders

# --- stand: a dark wooden chopping block, 250 x 150 x 40 mm, fixed to the table, a black silicone baking mat
# cut to size on its top (the mat is under 1 mm thick: it is the stand's top colour and friction) --------
STAND_SIZE = np.array((.250, .150, .040))
STAND_CENTRE = np.array((.400, 0., .020))
STAND_TOP = float(STAND_SIZE[2])
# How the stand is drawn (its collider is the plain 250 x 150 x 40 box): a block with 10 mm rounded upright
# corners and a 1.5 mm chamfer round its top, and on it the mat, 0.8 mm thick, cut 2 mm short of the block's
# edge all round so that a rim of wood shows. The mat's top is the collider's top.
STAND_CORNER = .010
STAND_CHAMFER = .0015
MAT_SIZE = np.array((.246, .146))
MAT_THICKNESS = .0008
MAT_CORNER = .008

# --- cake: Entenmann's All Butter Loaf Cake, 11.5 oz (326 g), out of its box and paper tray --------------
CAKE_SIZE = np.array((.185, .090, .060))       # length (x), width (y, along the wire), height; ESTIMATE (box 202 x 109 x 83)
TOP_LAYER_H = .030                             # the cut is half way up
BASE_H = float(CAKE_SIZE[2]) - TOP_LAYER_H
CAKE_MASS = .326                               # net weight on the box
CAKE_DENSITY = CAKE_MASS / float(np.prod(CAKE_SIZE))   # 326 kg/m3 with the estimated size
CAKE_HALF = CAKE_SIZE / 2
CAKE_CENTRE = np.array((.400, 0., STAND_TOP + CAKE_HALF[2]))
BASE_MASS = CAKE_MASS * BASE_H / float(CAKE_SIZE[2])
TOP_LAYER_MASS = CAKE_MASS * TOP_LAYER_H / float(CAKE_SIZE[2])
BASE_CENTRE = np.array((.400, 0., STAND_TOP + BASE_H / 2))
TOP_LAYER_CENTRE = np.array((.400, 0., STAND_TOP + BASE_H + TOP_LAYER_H / 2))
SEAM_Z = STAND_TOP + BASE_H                    # height of the cut above the table
SEAM_LOCAL = BASE_H - CAKE_HALF[2]             # and in the frame of the whole cake
CAKE_FRICTION = 1.0                            # cake on a silicone baking mat, ESTIMATE
# The knife score round the cake at the height of the cut: a V-shaped nick through the browned skin, which
# shows the pale crumb under it. Bakers score a cake like this before splitting it with a wire or a thread,
# and lay the wire in the score.
LINE_WIDTH = .0012                             # across the score, up the cake's side
LINE_DEPTH = .0008                             # into the cake
# How the cake is drawn. Its collider, and the cake the wire model cuts, are the plain 185 x 90 x 60 box; the
# drawn loaf lies wholly inside that box, so anything that touches the drawn cake has touched the collider.
# A loaf baked in a paper tray: upright corners rounded, sides leaning in toward the base, a rounded
# shoulder, and a top that rises to a ridge along the middle, split open along the ridge as butter loaves are.
# All ESTIMATES of an ordinary loaf's shape, not measurements of this product.
CAKE_CORNER = .014                             # radius of the upright corners
CAKE_BASE_INSET = .003                         # each side stands this far inside the box at the base ...
CAKE_TOP_INSET = .0004                         # ... and this far at the shoulder
CAKE_SHOULDER_Z = .043                         # the sides end here, above the mat
CAKE_SHOULDER = .007                           # radius of the rounded shoulder; the top starts at 50 mm
CAKE_CREST = .0585                             # height of the ridge beside the split; nothing is above 60 mm
CAKE_SPLIT_HALF_WIDTH = (.0012, .0065)         # the split along the ridge is 2.4 to 13 mm wide, ragged
CAKE_SPLIT_DEPTH = .0022

# --- cutting wire: Fante's Uncle Paolo's Cheese Wire, 12 in (model 2552) --------------------------------
WIRE_LENGTH = .305                             # 12 in between the handles (maker's page)
WIRE_DIAMETER = .0005                          # "fine-gauge" stainless steel; ESTIMATE
WIRE_LINEAR_MASS = .0015                       # kg/m, ESTIMATE from the diameter
WIRE_DRAWN_DIAMETER = .005                     # the cameras are shown a 5 mm tube; the wire that cuts is 0.5 mm
HANDLE_LENGTH = .065                           # wooden handles; ESTIMATE (not published)
HANDLE_RADIUS = .006                           # 12 mm; ESTIMATE (not published)
HANDLE_MASS = .005                             # ESTIMATE: 7.4 cm3 of hardwood at 0.65 g/cm3
HANDLE_FRICTION = .9                           # hardwood against rubber finger pads and the table; ESTIMATE
# The handles lie flat on the table, pointing away from the robot, the wire slack between them and bowed
# away from the robot.
HANDLE_START = {"left": np.array((.140, .130, HANDLE_RADIUS)), "right": np.array((.140, -.130, HANDLE_RADIUS))}

# --- cake acting on the wire ----------------------------------------------------------------------------
# Force to push a wire sideways through cake crumb, per metre of wire in the cake. A cylinder moving through
# a soft cohesive material is resisted by about 10.5 x shear strength x diameter (the T-bar factor of soil
# testing). For a dense butter loaf cake, shear strength about 3 kPa (ESTIMATE, from a crumb that takes
# 5 to 10 kPa to compress): 10.5 x 3 kPa x 0.5 mm = 16 N/m.
CAKE_SHEAR_STRENGTH = 3.e3
CAKE_YIELD = 16.                               # N per metre of wire
CAKE_VISCOUS = 120.                            # N s/m per metre of wire; ESTIMATE, +11% at 15 mm/s
CAKE_ALONG = .15                               # share of both that resists the wire sliding along itself
CUT_FORCE = CAKE_YIELD * float(CAKE_SIZE[1])   # 1.4 N to cut across the whole 90 mm width, slowly
CAKE_SLIDE_FORCE = CAKE_FRICTION * CAKE_MASS * 9.81    # 3.2 N to slide the cake on the mat

# --- names and world poses for the scene file -----------------------------------------------------------
ACTORS = ("base", "top_layer", "handle_left", "handle_right")
START = {"base": BASE_CENTRE, "top_layer": TOP_LAYER_CENTRE,
         "handle_left": HANDLE_START["left"], "handle_right": HANDLE_START["right"]}
MASS = {"base": BASE_MASS, "top_layer": TOP_LAYER_MASS, "handle_left": HANDLE_MASS, "handle_right": HANDLE_MASS}
