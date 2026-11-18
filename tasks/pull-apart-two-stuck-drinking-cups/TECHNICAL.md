# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The cups, and where each number comes from

Product: **IKEA KALAS tumbler, article 004.613.79**, a six-pack of polypropylene children's tumblers in mixed
colours (listed as "KALAS mug" in some countries). It is an open tapered cup with no handle. Any two cups from
one pack are used; the scene colours them blue (outer) and yellow (inner).

| Quantity | Value used | Source | Status |
|---|---:|---|---|
| Rim diameter | 80 mm | Retailer: "Diameter: 8 cm" [1] | Published |
| Volume | 23 cl (8 oz) | Retailer [1][2] | Published (not used by the model) |
| Material | Polypropylene | Retailer [1][2] | Published |
| Six cups stacked | 16.5 to 17 cm tall | Retailer package size: 17 cm [1], 6 1/2 in [2] | Published |
| Pack weight | 0.19 kg (7 oz) for six | Retailer [1][2] | Published |
| Price | USD 1.99 for six | Retailer [2], read 2026-11-16 | Published |
| Height of one cup | 81 mm | Third-party drawing: 8.1 cm [3] | Not confirmed by the retailer; the source marks its page "under development" |
| Mass of one cup | 30 g | Third-party drawing: 0.03 kg [3]; consistent with 0.19 kg / 6 = 32 g including the pack's wrapper | Not confirmed by the retailer |
| Base diameter | 66 mm | **Not published anywhere we found** | Estimate |
| Wall and base thickness | 1.5 mm | **Not published anywhere we found** | Estimate |
| Friction, cup on cup | 0.35 | Typical handbook range for dry polypropylene on itself | Estimate, not measured |

The two estimates were chosen together so that the model reproduces two published facts. Identical tapered
cups that touch wall to wall stack at a pitch of `wall / sin(half angle)`; with a 66 mm base and a 1.5 mm
wall the half angle is 4.94 degrees and the pitch is 17.4 mm, so six cups stack to 81 + 5 x 17.4 = 168 mm
(published: 165 to 170 mm). A polypropylene shell of that size and thickness weighs 30 g. Before a real
trial, measure the base diameter and wall thickness of a bought cup with calipers and, if they differ,
change `BASE_RADIUS` and `WALL` in `cup_model.py` and run `python -m rdb_nested_cups.build_scene`.

The rim is 80 mm across and the YAM gripper opens to 94 mm between its pads, so the cup fits between the
open fingers with 7 mm to spare on each side; the cup is not scaled.

[1] https://www.ikea.com/se/en/p/kalas-mug-mixed-colours-00461379
[2] https://www.ikea.com/us/en/p/kalas-tumbler-mixed-colors-00461379/
[3] https://www.dimensions.com/element/ikea-kalas-kids-tumbler



## Setup

Units mm. x forward from the line through the two arm bases, y to the robot's left, z up from the table top.
Arm bases at (0, +305) and (0, -305), 2.5 above the table top. Table top 1100 x 1300 (x -190 to 910,
y -650 to 650). Both arms start at the stock ready pose with grippers open.

| Object | Start pose (base centre) | Orientation |
|---|---|---|
| Outer cup, blue | (275, +35, 0) | Upright |
| Inner cup, yellow | (275, +35, 17.9) | Upright, same axis, inside the outer cup |

The simulation authors the outer cup 0.2 mm above the table and lets the scene settle for 0.5 s. At reset
the adapter checks both cup positions within 2 mm and both masses within 0.1 g, and records the settled
height of the inner cup above the outer one (`settled_pitch_mm`; authored 17.92, and the bond keeps it there).



## Real-world build

| Item | Detail | Source |
|---|---|---|
| Two cups | IKEA KALAS tumbler, article 004.613.79, six-pack, about USD 2; use two of the six | [1][2] |
| Table | Any flat wooden or laminate top at least 60 x 60 cm in front of the arms | - |
| Kitchen scale | 2 kg range, 1 g steps, to set the 10 N press | Any |
| Luggage or spring scale | 50 N range, to check the pull-apart force | Any |
| Tape measure, level or protractor, phone camera | For marking the start and scoring | Any |

Mark (275, +35) on the table. Stand one cup on the scale, set the other inside it, zero the scale and press
the inner cup straight down until the scale reads 1.0 kg; hold for 2 s. Move the pair to the mark. Check
that lifting the inner cup by its rim lifts both. Once per pair of cups, hold the outer cup down, hook the
luggage scale under the inner cup's rim (a loop of string round the rim band) and read the peak pull; adjust
the press so that it is between 5 and 7 N (the simulated bond breaks at 6 N). Use dry cups: a water film adds suction that the simulation does
not model. The cups survive many trials; replace one that is cracked or creased.

Safety: the cups are light, soft-edged polypropylene; nothing is sharp, hot, wet or heavy, the pull
is under 15 N and the finger squeeze under 40 N.



## Third-party assets and licences

- No third-party meshes, scans or textures are used for the cups: `cups.usda` is generated in this folder
  from the dimensions above. IKEA and KALAS are trademarks of their owner; the product is named only so the
  setup can be reproduced.
- Robot, station and cameras come from the [pinned YAM Lab-Arena environment](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1); its YAM arm model
  is from I2RT Robotics under the MIT licence (`LICENSE.i2rt` in that repository). That repository states
  that it does not yet carry a licence of its own for the rest of its code. Nothing from it is copied here.
- Isaac Sim and Isaac Lab-Arena are used under their own licences and are not redistributed.
- The code in this folder is under the MIT licence (`LICENSE`).



## Full simulation fidelity statement

Rigid bodies in Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control); every collider is rigid. The stuck fit is a rule: one bond between the two cups (a fixed joint) that breaks for good when the force between the cups passes 6 N, or the moment between them 0.3 N m (a 6 N pull at the rim is 0.2 N m); nothing else joins the cups and nothing moves them but the grippers. A friction wedge could not be used: with the walls pressed together on this simulator's contact springs, the inner cup crept out untouched within 7 s until little more than its own weight held it, in every run, because the contact friction that acted was about 0.05 and not the 0.35 set. So, unlike a real wedged pair, the bond does not loosen gradually, is not tightened or loosened by squeezing the outer cup, lets go all at once instead of over the first millimetre or two, breaks under a rocking moment as well as a pull, and does not re-form if the cups are pushed back together; there is no air or water-film suction; while bonded the rigid walls stand 0.04 mm clear of each other. Each cup collides as 24 flat wall panels of the true 1.5 mm thickness and a base disc, with two collision-only simplifications of the inner cup, which is drawn with its true 1.5 mm wall and keeps its outside shape, size and mass. First, its top 14 mm (the rim band above the outer cup) collides as 24 blocks with the true outside surface but 12 mm deep inward: the simulated fingers are position drives (5,000 N/m each) that cannot be commanded past touching, so a pinch on a 1.5 mm wall would squeeze with under 4 N, where a real gripper sets its force (about 40 N); on the 12 mm band the pinch squeezes with 30 N. Second, below that band its inside is solid (nothing has to go deeper into the cup than a fingertip, 13 mm), which gives the outer gripper something solid behind the walls it squeezes. So the inner cup's mouth is 56 mm wide for collision instead of 77 and only 15 mm deep, and a pinch of its rim closes on 12 mm instead of 1.5. Also simplified: the walls do not flex, so a squeeze cannot ovalise, crease or crack a cup; the 24-sided walls resist twisting one cup inside the other more than round cups do; friction is one estimated value, 0.35, for cup on cup, cup on table and finger pad on cup, below real rubber pads. The base diameter and wall thickness are estimates, and the 10 N press that seats a real pair to a 6 N pull-apart force is computed from the estimated friction and taper, not measured on real cups.
