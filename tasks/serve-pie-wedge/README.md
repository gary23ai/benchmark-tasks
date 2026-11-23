# Serve the first slice from a full pie pan

The operator uses a pie server to remove the first wedge of a chilled pumpkin pie. All eight cut portions begin inside a metal pan. The serving has a bottom and a curved outer crust, a filling and a narrow point that needs support over the gap to the dessert plate. The whole server starts outside the pan.

## Equipment and reset

Use the standard dual YAM station with gravity, rigid contacts, seed 0 and 20 Hz controls. The arms start in the standard ready pose with open jaws. Coordinates are in millimetres relative to the standard tabletop, with x forward, y left and z up. The compact-laminate work surface spans 880 by 1000 and is 4 thick. Its mass is 5.03 kg. An extension between the robot mounts supports the rear of the utensil. Secure the serving pan and dessert plate so they do not slide during the transfer.

| Item | Dimensions and starting position |
| --- | --- |
| Aluminum pan | The center is (520, −64, 4). The inside diameter at the bottom is 206.4, the top inside diameter is 228.6 and the sloped wall rises 19.05 above the pan floor. The intact flange extends to a diameter of 257.2. The metal is about 1 thick and the pan mass is 190 g. |
| First slice | Cut eight 45-degree portions through the filling and pastry. Keep all in the pan. Turn the selected wedge's curved side toward the operator along the negative x axis. The loose filling weighs 69 g. The bottom and raised outer crust together weigh 14 g. The bottom is 3.8 thick, the filling 18.2 high, and the outer pastry rises approximately 22 mm. At reset the center frames of the selected crust and filling are (455, −64, 6.9) and (455, −64, 17.9). The original filling and crust align at the point and the ends of the two cut faces. |
| Other seven portions | Each weighs approximately 83 g. Place the portions at 45-degree intervals around the pan center. Leave a nominal 2.5 mm slit at both cut faces of the target. At reset, verify it near the middle of each face with a thin ruler. The outer pastry stays against the normal flared pan wall. Remove any cutting guide. All seven neighbors rest on the pan floor and can move. |
| Server | A tapered 91 × 48 × 0.8 stainless blade (22 g) starts flat, tip at (365, −64, 4.5) facing forward. A raised neck joins it to an 85 × 19 × 17 hardwood grip. Grip and neck weigh 49 g together. Place the grip center at (211, −64, 36.5), with its rear resting on the open 10 × 28 support block. |
| Dessert plate | A 170 mm round, 5 mm thick glazed plate weighs 240 g. Place its lower center at (532, −309, 4). The nearest edges of pan and plate have around 31 mm of space between them. |

The server is a compact tapered offset design with a wooden handle. The manufacturer illustrates the corresponding full-sized form. [Tapered wood-handled server](https://www.amnow.com/product/19006/)

The 2.5 mm reset slit is specified and checked independently of the cutting implement. A common eight-inch kitchen knife has a blade thickness in this range. [Kitchen blade specifications](https://www.macknife.com/products/professional-series-8-chefs-knife-with-dimples-mth-80)

The pan dimensions follow the manufacturer's full-rim model. A kitchen demonstration shows why cutting both faces before the first lift helps. The utensil test describes supporting the wedge's point and heel on a tapered blade. [Pan specifications](https://www.amnow.com/product/1187/) · [First-slice demonstration](https://www.youtube.com/watch?v=MWLNtUKbXt4) · [Server evaluation](https://www.americastestkitchen.com/equipment-reviews/1901-pie-servers)

## Preparing the physical pie for repeat trials

Use a single batch of firm, plain pumpkin custard for a series. One reproducible formulation is the egg, evaporated milk and pure pumpkin filling in the manufacturer's recipe. Use plain pastry and omit whipped topping. Roll the base to about 4 mm. Line the specified shallow 9-inch pan, keep the outer pastry about 22 mm high, and portion the filling so the baked layer is about 18 mm deep. Reserve excess batter. Use the recipe's near-centre clean-knife test to check that the custard is fully set. A shallow pan can finish sooner than the deep pan the recipe describes. [Manufacturer's pumpkin custard recipe](https://www.verybestbaking.com/libbys/recipes/libby-s-famous-pumpkin-pie/)

For comparable trials, refrigerate the baked pies within two hours and hold them overnight in a refrigerator set to 3–4 °C. Start each trial with the pie at 3–4 °C. Measure at the back of a non-target portion with a clean probe. Use a clean, dry knife to cut eight equal sectors, leaving the bottom and side pastry attached to their original sectors. Remove the knife and cutting guide. Check the two cuts alongside the target near mid-radius with a thin ruler. The target starts flush with its neighbours, with an empty server outside the pan. Use a fresh complete pie with all eight portions for each first-slice trial and a clean, dry server and plate. These are physical reset specifications. The simulator starts directly in the stated chilled, cut configuration. [USDA refrigeration guidance](https://ask.fsis.usda.gov/article/At-what-temperature-should-perishable-foods-be-stored) · [USDA cooling guidance](https://ask.fsis.usda.gov/article/How-do-I-handle-leftovers-safely/)

## Recreating the score on a real serving

Record a level side view and an overhead view. Include a ruler at plate height and one beside the pan. Both jaws, the first wedge and the entire server should be visible. Compare the point and both outer ends of the filling to their original positions over the bottom crust. A drop, an empty blade or the filling leaving its shell does not count as a complete lift. Contact between either gripper and the target food voids the attempt.

| Credit | Observable event |
| ---: | --- |
| 0.25 | The server supports the whole first slice. The entire bottom crust clears the pan lip by 5 mm for at least 0.25 seconds. Both layers are upright within 15 degrees and their three corresponding landmarks remain within 9 mm. |
| 0.50 | Following the lift, the loaded server carries the entire outlined slice 5 mm past the outer flange on the plate side for at least 0.5 seconds. The same alignment and upright limits apply. |
| 0.75 | Lower the server until plate and blade briefly support the whole aligned serving together. The plate then supports the bottom crust, the entire serving falls within the plate edge, and the aligned layers are upright within 7 degrees for at least 0.25 seconds at the end. |
| 1.00 | The plated condition holds throughout the final second, and the edge of the blade stays at least 5 mm clear of the projected outline of both layers. The overhead view resolves a blade nose that still protrudes under the serving. |

The real handoff requires the blade and plate to carry the slice together. A physics engine can stop reporting forces after an object falls asleep. After the measured handoff the grader also recognizes the resting underside at the plate plane. Six bottom-crust points, including its tip and the rear arc, must all lie within 0.35 mm of the plate surface. This tolerance belongs to the numerical contact test. On a real plate, watch the side view for daylight and the overhead view for a tip extending past the rim. A lifted heel, a raised point or an intact slice left hovering fails the flat-rest check.

Inspect the two recordings frame by frame, including the last second. For an insertion control, angle the blade into the gap inside the outer crust and lift only the filling. A second control inserts the server shallowly under the heel. These trials test whether the original base, outer pastry and filling actually travel together. For a release control, leave part of the blade under the fully plated wedge.

## Physical scope

A rigid stainless server, complete flared pan wall, raised pastry and all eight cut portions participate in contact. The original target crust and filling can translate, rotate, separate and fall. Other portions approximate firm cut solids. Friction values are estimates. Within-layer cracking, crumbs, adhesion to the pan, local flexing of the blade and deformation of warm or loose filling lie outside this model. Use fresh, firmly set slices for physical repetitions. Count a visibly broken or missing target piece in the real trial as an incomplete serving and document that additional failure mode.

## Running the task

From the task folder inside the pinned YAM/Inspect environment, run the commands below. The declared horizon is contained in task.json. The robot receives RGB and joint state, then commands the joint targets and opening of the standard jaws.

```shell
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

The native geometry recorder probes each finite ramp on the collision mesh. If a reset-time scene ray has no corresponding hit, it intersects the composed, enabled convex USD collider in the rigid body's own frame. This keeps the mesh and measurement in the same pose after the server settles.

## Referee camera views

The recorded view includes the three stock station cameras and four fixed optical views. The policy receives the stock observations. Camera positions and look-at targets are expressed in metres in the station frame, with elevation measured from its table datum. Lens focal lengths are in millimetres.

- First slice / overhead camera at [0.463, -0.069, 0.416], aimed at [0.465, -0.064, 0.021], focal length 33.0 mm.
- Pan rim / side camera at [0.245, 0.22, 0.091], aimed at [0.462, -0.064, 0.044], focal length 27.0 mm.
- Dessert plate / overhead camera at [0.53, -0.311, 0.416], aimed at [0.532, -0.309, 0.025], focal length 22.0 mm.
- Plate and server / side camera at [0.53, -0.65, 0.084], aimed at [0.532, -0.309, 0.038], focal length 27.0 mm.

## Reading the native demonstration

The original crust and filling clear the pan at 18.65 s, pass its outer flange at 21.40 s and first share support between plate and blade at 22.65 s. The final side view shows the lower crust resting flat. During the last second, the projected outline stayed at least 11.6 mm inside the dessert plate and the server stayed at least 108.2 mm clear of both layers. The final measured crust-to-plate load was 0.814 N.

All 91 tapered collision components passed both surface-height probes at the original tolerances. Contact channels measure the working blade and bottom crust, the receiving plate and the grippers touching the target food. The adjacent portions have collision geometry but no separate load cells. The general all-bodies instrumented-contact summary in validation.json includes those neighbours; its detailed entries distinguish a measured force from an uninstrumented body.

## Recorded evaluation

Dream-1 run: 0 (627 steps); demo: 1.0 (627 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 409.49 seconds for the demo and 433.85 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

An earlier Dream-1 session expired. The submitted model video records one complete new trial from the same initial scene, seed and control horizon.
