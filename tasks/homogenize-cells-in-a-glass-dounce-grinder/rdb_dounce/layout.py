"""Every dimension of the grinder, the cold block and the lab jack, the liquid law, and the geometry the scorer uses.

Units: metres, kilograms, seconds, newtons. Each number is marked:
PUBLISHED  read on a seller's page for the named product (README gives the page);
COMPUTED   worked out here from published numbers or physics;
ESTIMATE   no maker publishes it; whoever builds the bench measures it and puts it here.

Frames. The tube's own frame has its origin on the axis at the outside of the tube's bottom, z up the axis. The
pestle's own frame has its origin on the axis at the very bottom of its head, z up the shaft.
"""

import math

import numpy as np

# ---- the Dounce tissue grinder: Wheaton 357542, 7 mL, two glass pestles -------------------------------------
TUBE_OD = .013                  # PUBLISHED: 13 mm outside diameter
TUBE_L = .082                   # PUBLISHED: 82 mm tube length
PESTLE_L = .175                 # PUBLISHED: 175 mm "overall length" (taken as the pestle's length)
CLEARANCE_B = .00005            # PUBLISHED: tight pestle B, 0.05 +/- 0.025 mm
CLEARANCE_A = .000114           # PUBLISHED: loose pestle A, 0.114 +/- 0.025 mm (not used in this task)
TUBE_ID = .0105                 # ESTIMATE: bore; 7 mL nominal over an 80 mm bore gives 10.5 mm (COMPUTED check below)
BOTTOM_T = .002                 # ESTIMATE: glass under the bore
RIM_OD = .0142                  # ESTIMATE: fire-polished rim bead, 1.6 mm tall
RIM_H = .0016
GLASS_DENSITY = 2230.           # borosilicate 3.3, kg/m3 (handbook)
HEAD_D = TUBE_ID - CLEARANCE_B  # COMPUTED: head of pestle B, 10.45 mm
HEAD_L = .013                   # ESTIMATE: ground head, a short cylinder with a rounded end
SHAFT_D = .006                  # ESTIMATE: glass rod
KNOB_D = .014                   # ESTIMATE: the solid glass handle at the top, 14 mm across
KNOB_L = .030                   # ESTIMATE: 30 mm long, rounded top
BORE_H = TUBE_L - BOTTOM_T      # 80 mm from the bore's bottom to the rim

# ---- the sample ----------------------------------------------------------------------------------------------
LIQUID_ML = 3.5                 # 3.5 mL of yeast suspension in PBS with 5 % glycerol, at 4 C
LIQUID_KG = 3.5e-3 * 1.02
VISCOSITY = 1.8e-3              # Pa s: water is 1.57 mPa s at 4 C; 5 % glycerol raises it by about 13 % (handbook)
P_ATM = 101.3e3                 # Pa
P_VAPOUR = .8e3                 # Pa, water at 4 C

# ---- collision geometry (rigid; the liquid law below stands in for the 0.025 mm film) --------------------------
STAVES = 16                     # tube wall: 16 boxes ...
STAVE_PHASE = math.radians(30.)  # ... turned so that two flats face the holding hand's pads (closing at 30 degrees)
WALL_OUT = TUBE_OD / 2          # stave outer faces on the true outside surface (apothem)
WALL_IN = TUBE_ID / 2           # stave inner faces on the true bore (apothem)
HEAD_R_SIM = .0046              # the head's collider: 0.65 mm radial play in the bore (real: 0.025 mm)
SHAFT_R_SIM = SHAFT_D / 2
KNOB_R_SIM = KNOB_D / 2
CONTACT_OFFSET = .001

# ---- cold block and lab jack ----------------------------------------------------------------------------------
BLOCK = (.075, .100, .040)      # ESTIMATE: aluminium cooling block, 75 deep x 100 wide x 40 tall, 4 x 5 wells
WELL_D = .014                   # ESTIMATE: wells for 13 mm tubes, 14 mm across
WELL_DEPTH = .025               # ESTIMATE
WELL_PITCH = .019               # ESTIMATE: well spacing
JACK = (.100, .100)             # ESTIMATE: 100 x 100 mm laboratory scissor jack (platform)
JACK_H = .075                   # set height of the jack's platform above the bench
SEAT_Z = JACK_H + BLOCK[2] - WELL_DEPTH     # the well's floor above the bench: where the tube's bottom sits
GRIP_Z = JACK_H + .084          # where the demonstration's left fingertip centre holds the tube (159 mm)

# ---- masses (COMPUTED) ----------------------------------------------------------------------------------------
def _cyl(d, length):
    return math.pi * d * d / 4 * length


TUBE_GLASS_KG = GLASS_DENSITY * (_cyl(TUBE_OD, TUBE_L) - _cyl(TUBE_ID, BORE_H))
TUBE_KG = TUBE_GLASS_KG + LIQUID_KG             # the liquid rides with the tube body
HEAD_VOLUME = _cyl(HEAD_D, HEAD_L - HEAD_D / 2) + 2 / 3 * math.pi * (HEAD_D / 2) ** 3     # cylinder + hemisphere
SHAFT_L = PESTLE_L - HEAD_L - KNOB_L
PESTLE_KG = GLASS_DENSITY * (HEAD_VOLUME + _cyl(SHAFT_D, SHAFT_L) + _cyl(KNOB_D, KNOB_L))
PESTLE_COM_Z = (HEAD_VOLUME * HEAD_L / 2 + _cyl(SHAFT_D, SHAFT_L) * (HEAD_L + SHAFT_L / 2)
                + _cyl(KNOB_D, KNOB_L) * (PESTLE_L - KNOB_L / 2)) / (HEAD_VOLUME + _cyl(SHAFT_D, SHAFT_L) + _cyl(KNOB_D, KNOB_L))
KNOB_Z = PESTLE_L - KNOB_L / 2   # the handle's centre in the pestle frame (where the fingers go)

# ---- the liquid law (a stated rule standing in for the fluid; README) ----------------------------------------
# Lubrication theory for a tight sphere moving along a tube (Bungay and Brenner 1973): drag
#   F = 6 pi mu a U * (9 pi / (4 sqrt 2)) * (h / a) ** -2.5,  a the head's radius, h the radial gap.
def drag_coefficient(radial_gap, viscosity=VISCOSITY, radius=HEAD_D / 2):
    return 6 * math.pi * viscosity * radius * (9 * math.pi / (4 * math.sqrt(2))) * (radial_gap / radius) ** -2.5


C_THEORY = {"0.0125 mm gap": drag_coefficient(.0000125), "0.025 mm gap": drag_coefficient(.000025),
            "0.0375 mm gap": drag_coefficient(.0000375), "0.05 mm gap": drag_coefficient(.00005)}
DRAG = 250.                     # N s/m used here: inside the theory's range, at its loose end (README)
HEAD_AREA = math.pi * (HEAD_D / 2) ** 2
SUCTION_CAP = (P_ATM - P_VAPOUR) * HEAD_AREA     # COMPUTED: the most an up-stroke can pull, 8.6 N: the liquid boils
FOAM_SPEED = .035               # m/s: above this the up-stroke reaches the cap (cavitation) and the sample foams


def fluid_force(v_rel, submerged=1.):
    """Axial force on the pestle (N, + up the tube) for the pestle moving at ``v_rel`` (m/s, + up) against the tube;
    the tube gets the opposite. ``submerged`` is the share of the head under the surface (0..1)."""
    f = -DRAG * v_rel * submerged
    return float(np.clip(f, -SUCTION_CAP, SUCTION_CAP))


# ---- the liquid's surface ------------------------------------------------------------------------------------
BORE_AREA = math.pi * (TUBE_ID / 2) ** 2
SHAFT_AREA = math.pi * (SHAFT_D / 2) ** 2


def surface_height(depth):
    """Height of the liquid's surface above the bore's bottom with the pestle's head bottom ``depth`` above it."""
    v = LIQUID_ML * 1e-6
    head_top = depth + HEAD_L
    below = BORE_AREA * head_top - HEAD_VOLUME          # liquid that fits up to the head's top
    if v <= below:                                       # head not covered: the surface is somewhere on the head
        return float(v / BORE_AREA + depth * 0)          # (approximate; only reached far outside the task's band)
    return float(head_top + (v - below) / (BORE_AREA - SHAFT_AREA))


def head_cover(depth):
    """How far (m) the head's top is below the surface; negative when the head is out of the liquid."""
    return surface_height(depth) - (depth + HEAD_L)


def submerged_share(depth):
    """Share of the head below the surface, 0..1 (linear over the head's length)."""
    return float(np.clip((surface_height(depth) - depth) / HEAD_L, 0., 1.))


# ---- the stroke bands (what the shaft's tape rings show at the rim) -----------------------------------------
BOTTOM_BAND = .005              # head's bottom within 5 mm of the bore's bottom: "down"
TOP_BAND = .027                 # head's bottom at least 27 mm up: "up" (head top then 14 mm under the surface)
COVER_MIN = .002                # head's top must stay at least 2 mm under the surface
# Tape rings on the shaft, measured from the head's bottom: each one is level with the rim at one depth.
RING_BOTTOM = BORE_H - BOTTOM_BAND          # at the rim: head 5 mm off the bottom
RING_TOP = BORE_H - TOP_BAND                # at the rim: head 27 mm up


def _limit_depth():
    lo, hi = 0., BORE_H
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if head_cover(mid) > COVER_MIN else (lo, mid)
    return lo


LIMIT_DEPTH = _limit_depth()                # head's top 2 mm under the surface: highest allowed head
RING_LIMIT = BORE_H - LIMIT_DEPTH           # the red ring: if it shows above the rim, the head is out
START_DEPTH = .030                          # the pestle stands in the tube with its head 30 mm up at the start

# ---- seating -------------------------------------------------------------------------------------------------
SEAT_LIFT = .003                # the tube counts as seated with its bottom within 3 mm of the well's floor
SEAT_TILT_DEG = 5.
SPILL_TILT_DEG = 30.            # tipped past 30 degrees, or lifted out of its well: the sample is lost
OUT_LIFT = WELL_DEPTH


def tilt_deg(rotation):
    return float(math.degrees(math.acos(max(-1., min(1., float(np.asarray(rotation)[2, 2]))))))


def tube_pieces():
    """Convex colliders of the tube in its own frame: name -> point cloud."""
    pieces = {}
    half = math.pi / STAVES
    for k in range(STAVES):
        a = STAVE_PHASE + 2 * math.pi * k / STAVES
        pts = []
        for r in (WALL_IN, WALL_OUT):
            w = r * math.tan(half)
            for s in (-w, w):
                for z in (BOTTOM_T * .5, TUBE_L):
                    x, y = r, s
                    pts.append((x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a), z))
        pieces[f"stave{k:02d}"] = pts
    # the bottom: a 16-sided slab under the bore
    ring = []
    for k in range(STAVES):
        a = STAVE_PHASE + 2 * math.pi * (k + .5) / STAVES
        r = WALL_OUT / math.cos(half)
        ring += [(r * math.cos(a), r * math.sin(a), 0.), (r * math.cos(a), r * math.sin(a), BOTTOM_T)]
    pieces["bottom"] = ring
    return pieces


if __name__ == "__main__":
    print("tube glass g", round(TUBE_GLASS_KG * 1e3, 2), "tube+liquid g", round(TUBE_KG * 1e3, 2),
          "pestle g", round(PESTLE_KG * 1e3, 2), "bore mL", round(BORE_AREA * BORE_H * 1e6, 2))
    print("drag theory N s/m", {k: round(v) for k, v in C_THEORY.items()}, "used", DRAG)
    print("suction cap N", round(SUCTION_CAP, 2), "speed at cap mm/s", round(1e3 * SUCTION_CAP / DRAG, 1))
    for d in (0., .002, .005, .027, .030, LIMIT_DEPTH):
        print(f"depth {d*1e3:5.1f}  surface {surface_height(d)*1e3:5.1f}  cover {head_cover(d)*1e3:5.1f}")
    print("rings from head bottom mm: bottom", RING_BOTTOM * 1e3, "top", RING_TOP * 1e3, "limit", round(RING_LIMIT * 1e3, 1))
