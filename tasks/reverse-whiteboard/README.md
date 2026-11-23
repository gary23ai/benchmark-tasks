# Reverse a meeting whiteboard

Turning the board exposes a clean writing surface while preserving a meeting chart. VIVO’s CART-WB24A manual places the rotation lock and marker tray on the mid support in step 3. Step 7 directs the user to push down the lock to rotate the board. The commercial board measures 24 × 36 inches. The smaller panel, pocket and spring specified here implement that operation on a bench. [Assembly manual](https://vivo-us.com/products/cart-wb24a) · [Product specifications](https://vivo-us.com/products/cart-wb24a)

## Station and reset

Use one task folder, `reverse-whiteboard`, seed 0, and the fixed dual-YAM PhysX tabletop at 20 Hz. Station dimensions below are in millimetres. The tabletop origin lies between the two bases. Positive x runs forward from the operator, positive y to the left, and positive z up. The left and right bases are at (0, 305, 2.5) and (0, −305, 2.5). All fixture bodies start unrotated.

| Object | Construction and mass | Starting layout |
|---|---|---|
| Stand | Bolted aluminum, 2.1 kg. Two 321 × 24 × 12 feet span 440 across their outer edges. Outboard bearing housings have 13 mm square openings. A 416 × 39 × 5 tray has a raised back lip. Two 20 × 7 × 40 guide cheeks flank the paddle link. | Frame origin (375, −40, 0). Axle supports at (375, −248 and +168, 155). Rear uprights at x = 518. Front edge of tray at x = 498.5. |
| Board | Balanced enamel-faced composite, 0.31 kg. Panel with edging measures 300 wide × 200 high × 6 thick. Each horizontal rail is 10 high. The two pivot stubs are 10 in diameter and 77 long. The blue chart is painted on one face. | Center (375, −40, 155). Chart toward −x. Original lower and upper rail tips at z = 55 and 255. |
| Clip | Acetal carriage, 0.0408 kg. Pocket opening 10 across the board thickness and 46 along the rail. Each retaining lip is 3 thick and rises 18 above the pocket floor. A 190 × 18 × 6 straight link connects to a 35 × 72 × 8 paddle. | Upper-stop origin (375, −40, 48). Pocket floor z = 51, lip tops z = 69. Paddle center (185, −60, 46). The lower rail starts between the lips with 2 mm clearance on each face. |

The carriage has 27 mm downward travel, a 260 N/m return spring and 4 N·s/m damping. Use an external guided spring and damper between the stand and carriage in parallel with the vertical slide through (375, −40). Place the spring outside the arm and rail sweeps. Index the upper and lower stops at carriage datums z = 48 and 21 mm. Tare the unloaded spring at the upper stop and verify the loaded return 1.54 mm below it before mounting the panel. Its calculated static drop under its own weight is 1.54 mm. A 24 mm depression places the lip tops about 10 mm below the rail and loads the spring to 6.24 N. The axle has zero restoring stiffness and 0.000045 N·m·s/degree damping. The panel corners sweep a radius of 100.05 mm. The tray begins 123.5 mm behind the axle plane. Each bearing housing begins 49 mm outboard of its panel edge.

Reset the original lower rail into the pocket with the chart toward −x. Release the paddle and let both bodies settle. Use the fixed runner’s standard arm home configuration with both grippers clear. The first commanded tip positions after home are (330, 125, 266) and (185, −120, 134), with jaw openings of 44 and 14 mm respectively.

## Manipulation

The scripted left gripper approaches the panel from its left edge at z = 232, enters 18 mm along y and closes across the two faces. The right gripper depresses the exposed paddle and holds its commanded pressure until the arriving rail is aligned. The extended link brings the paddle forward of the panel’s sweep, allowing the pressing hand to stay clear. Each arm can independently maintain contact while the other moves. The left follows a 77 mm radius around the axle in twelve 15° steps, rotating its closing direction with the panel. Once the board is near horizontal, the left hand opens and draws back past its free edge. It rolls the wrist half a turn in the open space, returns to the same edge and takes a second grip. The right hand keeps the paddle depressed. After the remaining half of the turn, the left holds the arriving rail aligned while the paddle rises, opens its jaws and withdraws sideways. The revised reference uses 576 steps and holds the final targets for the remaining 24 steps. Either direction of rotation, either hand assignment and a regrasp are eligible.

A shallow paddle stroke can leave the original rail trapped. A gripper can also travel around the intended arc after missing the edge. During the demonstration, check pad engagement during closure and rotation, wrist clearance from the bearings and bench, and separation after opening. The runner supplies gravity, rigid-body contact and passive joints. The policy observes RGB and robot joints and commands joint targets and normalized gripper width.

## Judge the released finish

The reference uses a fixed 20 Hz station and a 600-step horizon. Inspect samples 581–600. The blank face must face the operator within 8° of upright. Establish a calibrated front and overhead view around the visible front rail. The marked rail midpoint stays within 1 mm between every pair of readings. The two lip corners, rail tip and paddle stop remain visible. Full credit requires both tip corners between the lips, the tip midpoint 4–18 mm below their tops and the paddle at most 2.5 mm below its upper stop. A stable outside-lip finish receives half credit.

Leave the board and latch standing on the frame alone. Neither arm touches them during the inspection second, and both grippers have at least 5 mm of separation from them. Compare the smallest visible gap with a 5 mm scale in the same region, using the second view for the hidden side. The simulator measures separation from the gripper housings and fingertips to the board and clip collision surfaces. It also checks all links of both arms for direct contact. No point sampled only at the endpoint replaces the final-second record.

## Commands and evidence

From the task folder use

```text
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

In the final second of the simulator reference, the clean face stayed 1.15 degrees from upright. The minimum gripper-to-board gap was 6.79 mm. The clip was 1.58 mm below its upper stop, and the rail midpoint sat 12.40 mm below the lip tops. Both rail tip corners remained within the opening and neither arm touched the board or clip. The 24 scoring cases include an outside-lip finish, a final-sample hand intrusion and a rail that rocks back to the same endpoint. To reproduce the physical check, use the marked opening, the calibrated close view and the 5 mm gap scale described above. Use a separate trial to assess the released clip under an actual writing load.

## Recorded evaluation

Dream-1 run: 0 (600 steps); demo: 1.0 (600 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 264.15 seconds for the demo and 236.26 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```
