# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The objects, and where each number comes from

| Quantity | Value used | Source | Status |
|---|---:|---|---|
| Cutting wire | Fante's Uncle Paolo's Cheese Wire, 12 in, model 2552 | Maker's page [1], USD 5.99 | Real product |
| What it is sold for | Firm cheese, hard butter, cheesecake, and cutting cakes into layers | Maker's page [1] | Published |
| How it is used | Stretch the handles until the wire is taut, lay it against the food and pull | Maker's page [1] | Published |
| Wire length | 305 mm (12 in) | Maker's page [1] | Published |
| Wire | "fine-gauge" stainless steel; 0.5 mm used | Maker's page gives no gauge | Estimate; measure the wire you buy |
| Handles | Wood (the maker's page; some retailers list plastic) | [1] | Published |
| Handle size and mass | 65 mm long, 12 mm thick, 5 g | **Not published** | Estimate; measure |
| Cake | Entenmann's All Butter Loaf Cake, 11.5 oz (326 g) | Retailer listings [2][3] | Real product |
| Cake's box | 202 x 109 x 83 mm | Retailer's shipping dimensions [2] | Published (the box, not the cake) |
| Cake size | 185 x 90 x 60 mm | **Not published**; from the box, less its tray | Estimate; measure each cake |
| Cake's resistance to the wire | 16 N per metre of wire in the cake | 10.5 x shear strength (3 kPa) x wire diameter (0.5 mm) [4] | Computed from two estimates |
| Rate term | 120 N s/m2 (11 percent more at 15 mm/s) | - | Estimate |
| Friction, cake on a silicone mat | 1.0 | - | Estimate; check with a spring scale |
| Cake's shape and colour | Rounded corners, risen split top, browned top, paler sides | Common to tray-baked loaf cakes; not measured on this product | Estimate; drawn only |
| Stand | Wooden chopping block, 250 x 150 x 40 mm, dark (walnut or similar) | Any kitchen shop | Ordinary item |
| Mat | Black silicone baking mat cut to 246 x 146 mm | Any kitchen shop | Ordinary item |
| Work surface | Matt black self-adhesive vinyl film, 540 x 600 mm | Any hardware shop | Ordinary item; colour only in the simulation |

[1] https://fantes.com/fantes-uncle-paolos-cheese-wire-12-in (opened 2026-11-17)
[2] https://www.target.com/p/entenmann-39-s-loaf-all-butter-11-5oz/-/A-12921514 (shipping dimensions and weight, as returned by a web search on 2026-11-17; the page itself was not opened)
[3] https://www.shoprite.com/sm/planning/rsid/245/product/entenmanns-all-butter-loaf-cake-115-oz-id-00072030000213 (search result, not opened)
[4] The resistance of a cylinder pushed sideways through soft cohesive material is 9.1 to 11.9 times shear
strength times diameter, 10.5 being the usual value (the "T-bar factor" of soil testing: Randolph and Houlsby
1984; Stewart and Randolph 1994). Wire cutting of cheese is analysed the same way (Kamyab, Chakrabarti and
Williams, "Cutting cheese with wire", Journal of Materials Science 33, 1998, doi:10.1023/A:1017517332112). No
published wire-cutting measurement on cake was found; the 3 kPa is a guess for a dense butter cake.

### Is a two-handled wire the tool people use?

Partly, and the task text says so. The tools most home bakers use to split a cake are a long serrated knife
and a cake leveller (a wire stretched in a frame or bow, held by one handle, its feet sliding on the table).
A free wire or thread held taut in both hands is the third common method, and this maker sells its
two-handled wire for exactly that. We could not find a product sold only as a two-handled cake wire; searches
of baking-supply shops returned frame levellers. A frame leveller would make a different and easier task (the
frame keeps the wire tight and its feet set the height), so the two-handled wire was kept, with a real
product that is sold for cakes.

### Why the cake stays put, and by how little

Nothing holds the cake but its own weight on the mat, as in a kitchen. Drawing the wire through its 90 mm
width takes 16 N/m x 0.09 m = **1.4 N**; sliding the 326 g cake on a silicone mat takes
1.0 x 0.326 kg x 9.81 = **3.2 N**. A slow, steady draw therefore has a margin of about two, and a pull twice
as hard, a jerk, or a wire stretched too tight round the corners drags the cake. On bare wood or baking paper
(friction about 0.4) the cake would slide at 1.3 N, less than the cut needs, which is why a baker steadies the
cake with a hand or stands it on something that grips; here it is the silicone mat. The cake slides before it
tips. Both forces are small beside what the arms can do (the fingers squeeze each handle with 30 N), so the
difficulty is control, not strength.



## Setup

Units mm. x forward from the line through the two arm bases, y to the robot's left, z up from the table top.
Arm bases at (0, +305) and (0, -305). Both arms start at the stock ready pose with grippers open.

| Object | Start pose | Orientation |
|---|---|---|
| Black film, 540 x 600 | x 60 to 600, y -300 to +300, stuck to the table | - |
| Stand, 250 x 150 x 40, mat on top | centre (400, 0), top at z = 40, fixed | long side along x |
| Cake, 185 x 90 x 60 | centre of its base (400, 0, 40) | long side along x |
| Scored line | a knife nick about 1 wide and 1 deep along the cake's sides and ends at z = 70 (30 above the mat) | - |
| Left handle, 65 x 12 | middle at (140, +130, 6) | lying flat, pointing along x |
| Right handle | middle at (140, -130, 6) | lying flat, pointing along x |
| Wire, 305 long | fixed to the middle of each handle, slack on the table | bowed away from the robot, as far as x = 204 |

### How the height of the cut is marked

Bakers mark a cake before splitting it in one of two ways: toothpicks pushed in at the right height all
round, to guide a knife, or a shallow score cut round the side with a knife held at that height, in which a
wire or a thread is then laid and drawn through. Toothpicks would be swept out by a wire, so the score is
used here: about 1 mm wide and 1 mm deep, made with a knife held flat on a 30 mm block. It shows as a fine
pale line where the knife has gone through the golden skin to the crumb. It is not a gap between two slabs
and the cake is one piece until the wire has passed. No camera can read the height from it at the start (see
"What the cameras see"), so the instruction states it: half way up, 30 mm above the mat.

### What is drawn and what collides

| Thing | For contact and for the cut | As drawn |
|---|---|---|
| Cake | Two boxes, 185 x 90 x 30 mm each, one on the other | A loaf inside those boxes: upright corners rounded to 14 mm, sides 3 mm inside the box at the base and 0.4 mm at the shoulder, a 7 mm rounded shoulder from 43 to 50 mm, a top rising to a ridge at 58.5 mm that is split open (2 to 13 mm wide, 2 mm deep, ragged); nothing above 60 mm |
| At the height of the cut | The box's face | 1.3 mm inside the face (2.1 mm in the score) along the sides and ends; up to 6 mm at the four rounded corners |
| Score | None | A V-shaped nick 1.2 mm wide and 0.8 mm deep, in the crumb's colour, half on each layer |
| Cut faces | The plane where the two boxes meet | Flat, in the crumb's colour, seen only once the top layer is moved |
| Stand | A box 250 x 150 x 40 mm | A block with 10 mm rounded upright corners and a 1.5 mm chamfer, in brown, and a black mat 246 x 146 x 0.8 mm whose top is the box's top |
| Handles | A cylinder 65 x 12 mm | The same cylinder |
| Wire | 0.5 mm | A tube 5 mm thick |

The drawn cake lies wholly inside its colliders, so a gripper that touches what a camera shows has touched
the collider (and scores 0); the collider is also touched up to 6 mm before the drawn corner is, and up to
10 mm above the drawn shoulder. The wire model cuts the box: at the four corners it meets resistance up to
6 mm before the drawn cake. Colours are plain (no textures): top crust in three browns, darkest along the
ridge; sides and ends a paler gold, as the paper tray leaves them; crumb pale yellow. The shape and colours
are those common to tray-baked loaf cakes and were not measured on this product.

In the simulation the cake is two rigid bodies of 163 g that meet at the scored line. At reset the adapter
checks all four bodies' positions within 4 mm and their masses within 1 percent, and checks that the arm link
frames it uses to keep the wire out of the arms are the ones in the robot's description file.



## The wire and the cake

PhysX cannot cut a body, so stated rules stand in for the cake's response. They are in `wire.py` and are
exercised without a simulator by `tests/test_wire.py`.

- **The wire** is a chain of 64 stretch-only springs with no inertia, advanced by one implicit solve per
  240 Hz physics step. Its two ends are fixed to the middles of the two handles, which are ordinary rigid
  bodies. The pull of the wire is applied to the handles, so it reaches the arms only through the fingers'
  grip. No spring carries more than 40 N (beyond that it stretches at a constant 40 N), so nothing the wire
  touches is loaded beyond that.
- **The pull on a handle** is taken where the wire leaves the handle. Usually that is the first segment.
  Where the first nodes of the wire are pressed against their own handle, the handle's push on them and their
  pull on its middle are one internal force and cancel; only the tension beyond acts on the handle.
- **A handle that nobody holds** (not gripped between both finger pads) weighs 5 g and is moved by PhysX
  with the wire's pull as an outside force once per step, which is stable only for a soft spring. It is given
  the share of the pull that makes the taut wire it hangs on act like a 144 N/m spring: a tenth for the whole
  wire, less for a shorter taut length. Pulls under 2 mN (4 percent of its weight) are not passed on at all,
  in place of rolling resistance, which PhysX lacks. A held handle gets the whole pull.
- **What the wire cannot pass through:** the table and the stand, both handles (as capsules), and every
  collision piece of both arms and grippers (as its bounding box, padded by 4 mm). The wire is turned aside
  by these; the arms do not feel that contact.
- **Cake** holds a node of the wire, at the cake's surface or inside it, until the sideways push on the node
  exceeds 16 N per metre of wire; beyond that the wire creeps through against that force plus 120 N s/m2
  times its speed. A wire that is slack, or merely laid against the cake, does not cut.
- **The cut** is recorded on a 2.5 mm grid over the cake's plan, with the height at which the wire left each
  cell. Cells cut within 5 mm of the scored line count as cut on the line; cells cut at any other height
  count as spoiled.
- **The top layer** is held on by a spring whose strength is 50 N (3 kPa over 185 x 90 mm) times the share
  of the plan not yet cut on the line. Only cutting on the line frees it.
- **The cake** receives the wire's push, split between the two layers according to the height of the wire,
  so a hard or fast pull drags it across the mat.

The rule does not fire on proximity or a light touch: the robot has to hold both handles, stretch the wire
and push it through with the modelled force over the whole 185 mm.



## Real-world build

| Item | Detail | Cost |
|---|---|---|
| Cutting wire | Fante's Uncle Paolo's Cheese Wire, 12 in (model 2552) [1] | about USD 6 |
| Cake | Entenmann's All Butter Loaf Cake, 11.5 oz, one per trial | about USD 5 to 6 each |
| Stand | Wooden chopping block about 250 x 150 x 40 mm, taped to the table | about USD 15 |
| Mat | Black silicone baking mat, cut 2 mm short of the block's top all round | about USD 8 |
| Black film | Matt black self-adhesive vinyl, 540 x 600 mm, under the stand and the wire | about USD 5 |
| 5 N spring scale, kitchen scale, ruler, knife, chalk, phone camera | For checks, marking and scoring | - |

Per trial (2 minutes): take a fresh cake out of its box and paper tray, weigh and measure it, stand it on the
chalk marks, and score a line along its sides and ends 30 mm above the mat with a knife held flat on a 30 mm
block. Lay the wire out with the handles on their marks. After the trial, lift the top layer, measure both
layers against a ruler at the four corners and the middle of each long side, and brush the crumbs away.

Two checks with the spring scale, once per batch of cakes: (1) loop string round a spare cake and pull it
along the mat: it should start to slide between 2.5 and 4 N; (2) hook the scale to both handles through a
loop of string and draw the wire slowly through a spare cake held by hand: 1 to 2 N is expected. If the two
are closer than a factor of 1.5, use a stickier mat. Record both; the simulation's numbers are estimates until
this is done.

Food and safety: the wire is the only thing that touches the cake, and the grippers touch only the wooden
handles. Wash the wire between trials; do not eat trial cakes. The wire is thin and under a few newtons of
tension; keep hands away from the line between the grippers while the arms move.



## What is estimated

Published: the wire's length, what it is sold for and how it is used; the cake's brand, net weight and box
size. Estimated, to be measured on the real things before a real trial: the cake's own size (185 x 90 x 60 mm),
its resistance to the wire (16 N/m), the rate term, its friction on the mat (1.0), the wire's thickness
(0.5 mm), and the handles' size (65 x 12 mm) and mass (5 g). The margin between cutting (1.4 N) and sliding
(3.2 N) rests on two of these estimates; the spring-scale checks above measure exactly those two.



## What the cameras see

Sizes computed by projecting the scene into the station's calibrated cameras at the policy's 320 x 176 pixels
(`tests/test_visibility.py`; no renderer, so lighting and shadows are not included).

| Camera | What | Size |
|---|---|---:|
| Overhead (0.94 m up, looking 60 degrees down) | A handle on the film | 2.5 x 12.6 px |
| | The wire as drawn | 0.9 to 1.0 px across (0.1 px at its real thickness) |
| | The cake's top: golden brown on the black mat | about 17 x 29 px |
| | The split along the top, in the paler crumb colour | 1 to 2 px wide, about 13 px long |
| | The cake's near end (paler gold) | about 5 px tall; the score on it 0.1 px: not resolved |
| | The cake's long sides | not visible |
| | The stand's wooden front | 3.2 px tall; the rim of wood round the mat 0.4 px: not resolved |
| Wrist, at the start | The cake | only the far end of the side facing that arm, at the edge of the image |
| Wrist, gripper 65 mm above a handle | The handle; the wire beside it | 16.3 px across; 6.7 px |
| Wrist, at the grasp | The handle; the wire beside it | 27.5 px across; 11.4 px |
| Wrist, 120 to 200 mm from the cake's side | The score | 2.0 to 1.2 px wide, a pale line in the golden side |

What each policy camera shows, in words. Overhead: the table, the black film, both handles and the wire
between them, and the cake from above and a little from the front, as a rounded golden-brown loaf with a
pale split along its top, standing on a black mat on a brown block. Wrist cameras: at the start, the table
ahead of each gripper, with the far end of the cake's side at the edge of the image and the arm's
own handle out of view; over a handle, the handle and the wire; while the wire is drawn through with the grippers leaning forward, whatever of the cake's side
is ahead of the gripper, where the score is 1 to 2 pixels wide.

Contrast ratios of the authored colours (1 is none, 21 is white on black): wire on film 12.0, handle on film
9.0, wire over the mat 12.0, top crust on the mat 2.8 to 4.6, cake's side on the mat 6.4, split on the top
crust 3.7, wire against the top crust 4.3, wire against the cake's side 1.9, score on the cake's side 1.6
(a light nick, as a real one is). No camera shows the height of the score at the start, so the instruction
states it: half way up, 30 mm above the mat.



## Third-party assets and licences

- No third-party meshes, scans or textures are used: `cake.usda`, including the meshes of the loaf and the
  stand, is generated in this folder by `build_scene.py` from the dimensions above, with plain colours. Product names are given only so the setup can be reproduced.
- Robot, station and cameras come from the [pinned YAM Lab-Arena environment](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1); its YAM arm model
  (including the collision pieces whose bounding boxes `arm_shapes.py` reads at run time) is from I2RT
  Robotics under the MIT licence (`LICENSE.i2rt` in that repository). That repository states that it does not
  yet carry a licence of its own for the rest of its code. Nothing from it is copied here.
- Isaac Sim and Isaac Lab-Arena are used under their own licences and are not redistributed.
- The code in this folder is under the MIT licence (`LICENSE`).


## Full simulation fidelity statement

Rigid bodies in Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control). Cake cannot be cut in PhysX, so stated rules stand in for it. The cake is two rigid bodies, the top and bottom layers, that meet at the scored line. For contact and for the cut each is a plain box (185 x 90, 60 tall together); the loaf that is drawn, with rounded corners and a risen, split top, lies wholly inside the boxes, and at the height of the cut it is within about 2 mm of them except at the four rounded corners (up to 6 mm). The wire is a chain of 64 stretch-only springs without inertia, fixed to the middle of each handle, so its pull reaches the arms only through the fingers' grip on the handles; no spring carries more than 40 N; for the cameras it is drawn as a light grey tube 5 mm thick, ten times its real thickness, and the wire that cuts and collides is 0.5 mm. The wire cannot enter the table, the stand, the handles, the gripper fingers or the arm links (it is turned aside by them; the arms do not feel that contact). The cake holds the wire, at its surface and inside, until the wire's sideways push exceeds 16 N per metre of wire in the cake (1.4 N across the cake), plus 120 N s/m2 times its speed; where the wire has passed is recorded on a 2.5 mm grid with the height of the cut, and the wire's push is applied to the cake, so a hard or fast pull drags it over the mat (friction 1.0, 3.2 N). Uncut cake holds the top layer on with up to 50 N, falling with the area cut on the line; a cut at any other height frees nothing. Simplified or uncertain: the cake does not compress, crumble or tear, and the fingers cannot dent it; the wire has no bending stiffness and stretches more than steel (1,500 N/m end to end); a handle that nobody holds is given a tenth of the wire's pull or less, and none below 2 mN (for numerical stability, and in place of the rolling resistance PhysX lacks); the stand is a plain box for contact, drawn as a wooden block with the mat on it; the black film and the mat are colour and friction only. The cake's size, cutting resistance and friction, the wire's thickness and the handles' size and mass are estimates and have not been measured. Nothing here has been run on hardware.
