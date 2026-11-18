# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The cleaner, and where each number comes from

Product kind: a **double-sided magnetic window cleaner for 3 to 8 mm single glazing**, the common triangular
type sold by many sellers under many names for about USD 10 to 20: two triangular halves, each with a wiping
pad on one face and an arched handle on the other, magnets inside, a safety cord on the outside half and a
plastic spacer to keep the halves apart in storage. No single maker's model is claimed here, and no part
number is given because we could not confirm one. The outline size is taken from one retailer's listing [1].

| Quantity | Value used | Source | Status |
|---|---:|---|---|
| Glass thickness rating | 3 to 8 mm single glazing | Listings for this type [1][5] | Published |
| Outline of a half | 150 x 130 mm triangle | Retailer: "13x15x4.5 cm" [1] | Published; that the outline is a plain triangle with these as base and height is our reading |
| Thickness of a half with its handle | 45 mm | Same figure [1] | Published figure; that it is one half and not the stored pair is our reading (ESTIMATE) |
| Body thickness (pad plus shell) | 22 mm | Not published | ESTIMATE |
| Handle bar | 80 x 24 x 10 mm, 13 mm gap under it | Not published | ESTIMATE |
| Mass of a half | 200 g | A comparable two-piece cleaner is listed at 440 g for the pair [2]; a boxed triangular one at 500 g [5] | ESTIMATE |
| Magnets | Two per half, 70 mm apart, pad faces of the two halves attract | A comparable product is described as having two rare-earth magnets per piece [4] | ESTIMATE of layout |
| Pull through 4 mm glass | 17.7 N | Not published by any seller we found | ESTIMATE (see below) |
| Pad on glass friction | 0.25 | Typical for dry microfibre on glass | ESTIMATE, not measured |
| Materials | ABS shell, microfibre pad, rare-earth magnets | Reviews and listings [3][4] | Published for comparable products |

The handle bar is 24 mm wide and the YAM gripper opens to 94 mm between its pads, so the gripper takes the
half by its own handle with 35 mm to spare on each side. Nothing is scaled and no grip aid is added. A larger
branded cleaner of the same idea (about 245 x 100 x 50 mm for the pair [2][3]) was considered and not used:
its body is wider than the gripper opens and its grip dimensions are not published.

**What is true of real cleaners, and what is not modelled.** The two halves are a matched pair whose pad
faces attract; they are sold stuck together with a spacer between them, and reviewers warn that the halves
are hard to pull apart if they meet without the spacer and that letting them snap together through a pane
can crack it [4][7]. There is no polarity to find out by feel: the pads attract. Whether real pairs
also have to be turned a particular way round in the plane of the glass is not published; in this model the
pads attract in any in-plane orientation, and they also catch weakly when shifted by one magnet spacing
(70 mm), which does not count as paired.

**How the magnet strength was set.** No seller we found publishes a holding force. The model is therefore
sized from the rating: through the thickest rated glass (8 mm) the halves must still pull together hard
enough that pad friction carries a released half with a margin, 10 N for a 0.2 kg half at friction 0.25
(2.5 N of friction against 2.0 N of weight). With that one number fixed, the model gives:

| Gap between the pad faces | Pull |
|---:|---:|
| 0 (pad to pad, no glass) | 32.4 N |
| 4 mm (this pane) | 17.7 N |
| 8 mm (rated maximum) | 10.0 N |
| 12 mm | 6.0 N |
| 20 mm | 2.5 N |
| 30 mm | 1.0 N |

Sideways, the pull back towards alignment peaks at 9 N about 10 mm off centre. A real cleaner may well be
stronger than this estimate. Before a real trial, measure the pull (see "Real-world build") and set
`HOLD_AT_RATED_N` in `cleaner_model.py` so that `hold_force()` matches; measure the half's outline, body
thickness, handle and mass with calipers and a kitchen scale and change the constants at the top of that
file; then run `python -m rdb_window_wall.build_scene` and the tests.

Even pad to pad the modelled pull (32 N) is below what two 40 N fingers can hold, and sliding the pair on
the glass takes under 10 N.

[1] https://www.mediamarkt.nl/en/product/_double-sided-magnetic-window-cleaner-120139366.html (the page
    refuses automated readers; the figures were read from a search result summary on 2026-11-16 and should be
    checked by hand)
[2] https://www.mydeal.com.au/glider-s-1-magnetic-window-cleaner-2-8mm-3245408 (440 g; 25 x 10 x 5 cm; read
    from a search result summary on 2026-11-16)
[3] https://the-gadgeteer.com/?p=248132 (2 x 9.6 x 4 in; ABS, rubber, rare-earth magnets; single-pane model
    for 0.08 to 0.3 in glass)
[4] https://www.coolthings.com/?p=89063 (two magnets per piece; spacer for storage; larger piece inside)
[5] https://hyperli.com/products/double-side-magnetic-glass (3 to 8 mm single glass; package 14 x 14 x 8 cm,
    500 g; read from a search result summary on 2026-11-16)
[6] https://tyrolerltd.com/glider-instructions (inside unit is moved, outside unit follows; safety cord)
[7] Buyers' reviews quoted in a reseller listing, https://www.desertcart.com.cy/products/95888846 (advice
    to slide the halves together along the glass so that they do not hit it; read from a search result
    summary on 2026-11-16)



## Setup

Units mm. x forward from the line through the two arm bases, y to the robot's left, z up from the table top.
Arm bases at (0, +305) and (0, -305), 2.5 above the table top. Table top 1100 x 1300 (x -190 to 910,
y -650 to 650). Both arms start at the stock ready pose with grippers open. The code also uses pane
coordinates (u along the wall, equal to y; n square to it, positive indoors, measured from the middle of the
glass at x = 410; z up).

| Object | Size, material, mass | Pose (tabletop) |
|---|---|---|
| Wall section | White-painted plywood 12 thick, 1280 wide, 560 tall; two feet 60 x 300 x 40 under its ends, clamped | Indoor face at x = 400, outdoor face at x = 412; y -640 to +640; fixed |
| Window cut-out | 990 wide x 320 tall | y -380 to +610, z 130 to 450 (sill 130 above the table) |
| Opening light | Clear opening 430 wide x 320 tall, sash left off | y -380 to +50 |
| Stile | 10 wide, 12 deep (part of the plywood) | y +50 to +60 |
| Fixed light | Clear glass 550 x 320 x 4, 1.76 kg | y +60 to +610, z 130 to 450; outdoor face flush with the wall's (x 408 to 412); fixed |
| Marker band | Black dry-wipe marker, 150 long x 30 tall | Outdoor face of the glass, y +410 to +560, z 280 to 310; 350 from the stile |
| Inside half, blue | Triangle 150 base x 130 height, 22 thick, handle top at 45; 200 g | Indoors, pad down, pad centre (centroid) at (260, +330), handle bar parallel to the wall, apex towards the wall |
| Outside half, white | The same | Indoors, pad down, pad centre at (280, +30), handle bar parallel to the wall, apex towards the robot |

The wall's indoor face is 63 mm beyond the fingertips of the arms' ready pose (they reach x = 337). The band
is 280 to 310 above the table because lower down the left arm, at its ready pose, hides that part of the pane
from the overhead camera.

### Reach: why the band cannot be wiped with a hand-held half

Checked without a simulator, on the arm's URDF kinematics with its collision hulls (sampled every 8 mm)
against the wall, stile and glass:

- A search over arm poses (collision-free starts outdoors by inverse kinematics from random seeds, then hill
  climbing on how far along the wall the gripper's centre point gets; any hand orientation; the wall given no
  top and no ends) ends at **y = +220** for the right arm in 8 of 12 runs (the others lower) and at y = +142
  for the left arm. At that limit the right arm is stretched nearly straight through the opening and its
  wrist is against the stile.
- However a half is held, no point of its pad is more than 130 from the gripper's centre point (bar held at
  its end, pad's far corner). So a hand-held pad ends at y = +350 at the very most, **60 short of the band**
  (which starts at +410); with the grip the demonstration uses, 90 short.
- An earlier, more permissive search (hull vertices only, which lets a link sink a little into the wall)
  ended at +230; the band is 50 beyond what that would allow.
- This is a numerical search, not a proof. The scorer does not depend on it: a square of the band counts only
  when the outside half wipes it with no gripper touching that half, and a gripper that goes over the wall or
  round its end scores 0. (Physically the left arm can get round the end of this 1280 mm wall section; a real
  wall has no end to go round.)

The demonstration sets the outside half down with its centre at y = +170 (gripper centre at +145, 75 short of
the limit above), every joint 0.25 rad or more inside its limits.

### What the policy's cameras see at the start

320 x 176 each, read off the first frames that Dream-1 received (`dream-input-rgb.zip`, frame 0) and checked
against a projection of the scene through the cameras' calibration.

- **Overhead camera** (behind and above the arm bases, looking forward and 60 degrees down): both arms; the
  blue half left of centre and the white half in the centre, both unobstructed (handle bars visible); the
  wall from the table up to the window's head, with the open light on the right (the table beyond shows
  through it) and the glass on the left, tinted grey; the marker band through the glass as a dark grey stroke
  at the upper left (projected: columns 26 to 64, rows 45 to 50), just above the left arm's wrist and clear
  of it. The outdoor face itself is not in view; the band is seen through the glass.
- **Left wrist camera**: the inside of the fixed light at close range: the glass, the table beyond it, a
  blurred dark horizontal shadow, the white sill and the fingertips; the tip of the blue half shows as a few
  blue pixels at the bottom edge. The band is not identifiable in it.
- **Right wrist camera**: the wall's face and sill below the open light, with the table beyond, and the
  fingertips; the white half is not in its view. It sees the outdoor face only once the hand is through the
  opening.
- The review (side) camera, which no policy sees, stands outdoors beyond the fixed light's far end and looks
  back along the outdoor face; the band shows on it as a crisp black stripe on the glass.

The simulation authors each half 0.3 mm above the table and lets the scene settle for 0.5 s. At reset the
adapter checks both positions within 4 mm and both masses within 0.1 g.



## Real-world build

| Item | Detail | Source |
|---|---|---|
| Magnetic window cleaner | Double-sided, triangular, for 3 to 8 mm single glazing, about USD 10 to 20; take the safety cord off the outside half | Hardware and online shops; outline as in [1] |
| Wall section | 12 mm plywood 1280 x 560, painted white, with a cut-out 990 x 320 whose bottom edge is 130 above the table and whose right-hand end (seen from indoors) is 260 from the wall's right end; a 10 x 12 mm stile fixed across the cut-out 430 from that end; two feet 60 x 300 x 40 screwed under the wall's ends; four G-clamps | Timber merchant |
| Pane | Clear sheet 550 x 320 x 4 mm: toughened glass with ground edges, or cast acrylic (which scratches more easily), bedded in the cut-out against the stile with glazing tape or beads so that its outdoor face is flush with the plywood's | Glazier or plastics supplier |
| Marker | Black dry-wipe (whiteboard) marker, chisel tip | Stationer |
| Luggage or spring scale | 50 N range, to measure the pull | Any |
| Ruler, calipers, kitchen scale, phone camera, a 5 mm grid printed on clear film | For setup and scoring | Any |

Stand the wall across the table with its indoor face 400 from the line through the arm bases and its ends
640 either side of the centre line, and clamp the feet. Check that the arms' ready pose clears it. Draw the
band on the outdoor face against a ruler, 350 to 500 mm from the stile and 280 to 310 mm above the table,
and let it dry. Lay the halves on their marks. Once per cleaner: put the halves on the pane opposite each
other, tie a loop of string round the outside half's handle, hook the scale to it, pull straight away from
the glass and read the peak; set `HOLD_AT_RATED_N` so that the model's 4 mm figure matches. Also check that
sliding the inside half by hand at walking pace drags the outside half along on the dry pane; if the pads
stick, breathe on the glass or use the pads the cleaner came with slightly damp, and note it.

Safety: the halves pinch. Keep fingers out from between them, keep the spacer between them in storage, and
do not let them snap together through the pane. Use toughened glass or acrylic, never plain float glass.
The modelled pull is under 20 N through the pane and the finger squeeze under 40 N; a stronger real cleaner
should be checked against the arms' limits before a trial. Put a mat on the table beyond the wall: that is
where a dropped outside half lands.



## Third-party assets and licences

- No third-party meshes, scans or textures are used: `window.usda` is generated in this folder from the
  dimensions above. Product and shop names appear only as sources for dimensions.
- Robot, station and cameras come from the [pinned YAM Lab-Arena environment](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1); its YAM arm model
  is from I2RT Robotics under the MIT licence (`LICENSE.i2rt` in that repository). That repository states
  that it does not yet carry a licence of its own for the rest of its code. Nothing from it is copied here.
- Isaac Sim and Isaac Lab-Arena are used under their own licences and are not redistributed.
- The code in this folder is under the MIT licence (`LICENSE`).


## Full simulation fidelity statement

Rigid bodies in Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control). The wall, stile, feet and glass are fixed box colliders and the open light is a real hole between them; the glass cannot crack or flex. Each half is one rigid body: two triangular prisms (pad and shell) and a handle bar on two posts, grasped by real finger contact. Nothing attaches the halves to the glass or to each other: each magnet is two softened point poles, and the force and torque on each half are recomputed and applied at every 240 Hz physics step, so the halves hold, slide and let go by contact and friction alone. The magnet strength is an estimate, not a measurement: no holding force is published for this kind of cleaner, so the model is sized to pull 10 N through the thickest rated glass (8 mm), which gives 17.7 N through this 4 mm pane, 32 N pad to pad and 1 N at a 30 mm gap; a real cleaner may well be stronger. Pad on glass friction is 0.25 (estimate for dry microfibre; real use is with a wet pane). The marker is not a material: the scorer keeps a grid of 180 squares of 5 mm over the band and marks one cleaned when the pad covers its centre under the conditions in the scoring, and that drawn square is then hidden. Nothing but the scoring rule stops a gripper going over or round the wall section. The demonstration reads the simulator's poses of the two halves to aim and check its moves; it moves them only with the grippers. It puts the outside half on the way a person does on real glass: it sets it on the pane 55 mm below the inside half, where the pull is under 0.5 N, and slides it up until the halves pair; a half snapped straight on would cost nothing here only because the simulated pane cannot crack. The colliders have sharp corners and a straight handle bar on two posts where the real halves are rounded with an arched handle; the halves are drawn rounded with a separate pad and a D handle over the gripped bar, and the wall is drawn about 80 mm thick with a window frame, beads, architrave, window board and skirting: these are display only, with no collider. The pad does not compress, there is no safety cord and no water.
