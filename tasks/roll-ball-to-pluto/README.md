# Drop the Shoot the Moon ball into Pluto

Spread the rods to roll the ball toward the handles, then open the gap enough for the ball to fall into Pluto. The stock game calls the pocket nearest the handles Pluto and awards it the highest value. [WE Games 497018](https://woodexpressions.com/products/shoot-the-moon-a-1940)

## Station and reset

Dimensions are in millimetres. Set the tabletop to z = 0. Positive x points away from the arms and positive y points toward the left arm. The arms mount at (0, ±305, 2.5). All actor frames start aligned with the tabletop. The rail cylinder shapes have the inclinations listed below.

| Part | Fixture drawing coordinates and construction |
|---|---|
| Base and frame | The 550 × 300 × 9 plywood base spans x = 280–830, y = −150 to 150. Two 423 × 18 × 9 runners at y = ±54 support the frame. The fixed wood assembly weighs 1.65 kg. Its outer bounds are x = 340–784.5 and y = ±65. The handle-end wood face is at x = 340. |
| Deck and guides | The deck is at z = 42. Side lips reach z = 57. The far stop's inner face is at x = 775. The transverse guide is centered at x = 358. Its rail bearing face is at z = 78.83 and its outer stop faces at y = ±42. |
| Pocket row | Six mouths, approximately 46 across flats, are centered at x = 725, 658, 591, 524, 457 and 390 along y = 0. Four yellow marks surround Pluto at x = 390. Separate bonded felt-faced pads for Pluto and the preceding wells both have their tops at z = 22. The Pluto pad weighs 0.02997 kg and the five-well pad weighs 0.12985 kg. A 0.5 mm seam lies under the divider at x = 423.5. |
| Rail axes at their inward stops | Each steel rod and its links weighs 0.165 kg. The rod is 8 in diameter and 420.036 long. Their rear axes begin at (770, ±12.5, 80). Moving toward the player, each rod rises 0.4° and converges 0.6366° toward the centerline. The vertical hinges are at (758, ±12.367, 80.084). At the inward stops both hinge offsets are 0°. The left hinge opens to −4.15° and the right to +4.15°. |
| Grip centers at the inward stops | Left (327, +64, 83.093), right (327, −64, 83.093). Each wooden grip is 14.64 in diameter, 46.004 long and weighs 0.005 kg. Each 8 mm square transverse steel link sits at x = 340, z = 83. Its length is 52.422 and its midpoint is at y = ±30.289. Each rigid bond represents a concealed internal fastener; the external wood and bare-steel surfaces use distinct, non-overlapping collision shapes. A 14 × 8 × 8 axial connector joins each crossbar to its rod. The crossbar and axial connector are both bare steel. |
| Ball | The steel ball is 30 in diameter and weighs 0.111 kg. Start its center at (760, 0, 94.48) against the far stop. Its seated center is 37 above the tabletop. |

Bring both hinges to the inward stops and let the ball settle against the far stop. Start both arms at their stock station joint home (0, 1, 1, −0.6, 0, 0) radians with jaws fully open. The same physical reset applies to both policies. Tick 1 begins with the first policy action.

## Play and clearance

The reference brings the open tips above and outboard of both grips at mirrored targets (286, ±76.422, 143.191). It then approaches through (313, ±76.422, 83.191) and closes across the wood near (315, ±61.844, 83.191). The fingers extend toward the board and 15° downward. The jaw closing direction is in the x–z plane. These are target poses, with a commanded 34 mm open approach and a 13 mm grasp target. The reference opens the hinges in two steps, waits for the ball to settle, opens its jaws and withdraws. Other grasps, opening rates and strategies can earn the same score if they meet the outcome and contact rules.

The ideal pass-through spacing is 38 mm between the rail axes. Opening too early can drop the ball into Saturn, even if the handles eventually reach the same positions as a successful turn. The offsets give the two pads room to reach the wood. Use the footage and measured pad poses through the approach, hold and withdrawal to confirm that the complete pads clear the nearest steel edges and each other. The signed clearance check covers both link sweeps, including the outward travel limits.

## Read the result

Use the tabletop game's usual handle-only play. Start two synchronized close recordings before the arms move. Place one camera above the pockets so that the drop location and handle use remain visible. Aim the other across the pocket openings so that the ball's first loaded pocket floor is visible. Film the full turn using the same views for a Pluto drop, an early-pocket drop and a lip glance. The opening edge is not the pocket floor. If a hand or rail covers the first landing in one angle, use the other angle. Use an additional close view of the narrow handle-to-steel junction if both station views leave it hidden.

Score 1 only when Pluto is the first floor the ball lands on and the ball remains in Pluto, supported by its floor and away from both rods, for the final second. At the final seated height place a 2 mm reference beside the narrow field of view and confirm support in the simultaneous side view. Take the largest three-dimensional span between any two centers in the last 20 readings. Up to 2 mm qualifies. Use the full positions, including a return to the original center. If a view cannot resolve the threshold or the original landing, record the uncertainty and improve the camera placement before a new scored trial.

Only the wooden grips are allowed robot contacts. A clear brush of the bare rods, ball, lining or frame gives zero even if the ball later lands in Pluto. A ball left on the rods, an outside miss or an earlier pocket floor gives zero. For simulation the reported signals use the actual contact channels at each physics step. The reference and controls are simulation trials. The apparatus, camera placement and judge agreement remain to be confirmed on a physical recreation.

## Reproduce

Use one folder named `roll-ball-to-pluto` on the pinned dual-YAM PhysX station. The two policies use the same joint reset, 320 × 176 RGB and joint observations, and the same 600-step limit at 20 Hz. Use seed 0. Run the bundled tests, then the scripted and Dream-1 policies in separate fresh output directories.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

Run the full 600-sample reference and earlier-pocket contrast with the same fixture and grading. The first contrast changes only the scripted release to an earlier opening of the rods. A second 600-sample contrast moves the left fingertips to the uncoated steel junction before ordinary manipulation, then continues on the same fixture and grader with a bounded approach and withdrawal. The first contact must remain recorded for the entire horizon. Compare the real force with the latched history at the first detected sample. Run a matched late-settling control with the same exact station. Begin from the fixed reset and delay the first arm movement while keeping the 600-sample horizon. On that contrast, a ball that seats in Pluto but moves more than 2 mm between any pair of the last twenty readings receives zero. The bench checks above describe a method for physical replication.

## Native scoring cross-check

The overview shows the handle use, the ball reaching the last pocket and withdrawal from the board. It is a wide view. Use replayed measurements for fine position checks in simulation and the calibrated close view above for a physical trial. The four runs below used the same simulated layout and differed in scripted actions. Scores shown use the submitted rules.

| Run | First pocket floor | Forbidden contact | Final-second ball span | Score |
| --- | --- | --- | --- | --- |
| Reference | Pluto at step 253 | None | 0.262 mm | 1 |
| Earlier pocket | an earlier pocket at step 151 | None | 0.072 mm | 0 |
| Bare-steel contact | an earlier pocket at step 145 | Step 68 | 0.712 mm | 0 |
| Late settling | Pluto at step 457 | None | 2.518 mm | 0 |

## Recorded evaluation

Dream-1 run: 0 (600 steps); demo: 1.0 (600 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 265.89 seconds for the demo and 259.4 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

The reference video is from the earlier native run with the identical scene and scripted controller. Its recorded measurements were replayed against the submitted scoring rules. The old scoring code returned 1. The submitted scorer returns 1 on that recording. The Dream-1 trial uses the submitted source.
