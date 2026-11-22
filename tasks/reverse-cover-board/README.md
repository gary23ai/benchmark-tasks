# Reverse one cover board to check the pair for squareness

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 600 | 226.98 s |
| Scripted demonstration | 1 | 600 | 237.98 s |
| Independent scripted repeat | 1 | 600 | 216.25 s |
| Hold initial pose | 0 | 600 | 211.5 s |

The design notes below were recorded before these native trials. The measured results are recorded in `validation.json`.

| Native simulation reading at step 600 | Scripted demonstration | Independent scripted repeat |
|---|---:|---:|
| Near-left endpoint separation in plan | 1.55 mm | 1.55 mm |
| Near-right endpoint separation in plan | 2.42 mm | 2.42 mm |
| Angle between the boards’ original face normals | 179.9999° | 179.9999° |
| Board-to-board normal force | 0.392 N | 0.392 N |
| Blue board-to-block normal force | 0.785 N | 0.785 N |
| Arm-to-board normal force, each of the four channels | 0 N | 0 N |
| Overview clearance, each required sample | 1.0 | 1.0 |

Both scripted runs finished within the 3 mm limit at each near endpoint and had the same trace SHA-256 at seed 0. At step 600 in Dream-1, the face normals were about 0.0012° apart and the ochre board center was still about 37.6 mm nearer the operator than the blue board center. The ochre near-right corner and its two adjoining edge samples each recorded 0 on the overview clearance metric. Dream-1 and the hold-initial-pose control passed none of the eight full-score alternatives.

The supplied demonstration frames show the gripper lifting and turning the ochre board, then show both arms clear and the far ledge visible at the finish. The 3 mm check uses the archived planar readings. The visual review covered the supplied frame excerpts. The full video and complete ZIP were outside the scope of independent verification.

<details><summary>Original pre-trial design record</summary>

# Reverse one cover board to check the pair for squareness

Task folder `reverse-cover-board`. A pair of cover boards cut together can agree along every edge while sharing a skew. Douglas Cockerell describes reversing one board after cutting so the inequality appears doubled. He advises saving a badly cut pair for a smaller book because correcting the error consumes material. The task ends with the loose pair ready for that inspection. This specialist bookbinding operation supports the Other category. [Cockerell, *Bookbinding, and the Care of Books*, chapter IX, after Fig. 49, pp. 131–132](https://www.gutenberg.org/cache/epub/26672/pg26672-images.html)

## Station and reset

All dimensions in this section are millimetres. The tabletop frame has +x forward from the operator, +y to the operator’s left and +z up. Bolt a hardwood plinth 140 deep × 240 wide × 50 high to the bench. Fit the 25 high block of the same footprint on top. Its flush cork face extends from x = 280 to 420 and y = −120 to +120 at z = 75. The assembly weighs 1.65 kg and has friction setting 0.78. The front lip has no fence.

Use two loose rigid millboards, blue and ochre, each 40 g with friction setting 0.35. Each has uniform 3 mm thickness, parallel 160 mm long edges and 100 mm depth. From the near-edge midpoint, the outline in (forward, left) coordinates is (0, 80), (0, −80), (100, −86), (100, 74). Two complementary collision wedges per board reproduce this outline. Their 3 mm width runs vertically, their flat bases meet at the longitudinal center plane, and each outside edge slopes 6 mm across the board depth. The pale front and dark reverse finishes each cover a centered 88 × 145 mm area, leaving the colored edge visible.

Set the blue body center at (330, −3, 76.5) and the ochre center at (291, −3, 79.5). Both actors start unrotated with the pale finishes up. The blue near corners are at (280, ±80) in plan and the ochre near corners at (241, ±80). The stack overlaps over 61 mm of depth. Remove the template after checking the four corners. Use the standard arm reset. The left and right base origins in metres are (0, +0.305, 0.0025) and (0, −0.305, 0.0025). Seed 0 is the reference seed. Object poses are fixed.

## Handling route and clearance

The scripted right hand approaches horizontally through the free space at the front. Its target at pickup is (271, 0, 79.5) mm. An 11 mm open jaw gives 4 mm nominal face clearance on each side of the stock. At closure the fingertips reach 30 mm past the projecting edge and end 9 mm before the lower board. The arm lifts the pinched board to z = 190 mm and rolls through six 30° increments about −x. The board sweeps a radius just over 86 mm, leaving about 26 mm above the blue board at the lowest point of that sweep.

Seat the board with the overhang still accessible. Its center of mass is over the overlap, so the fingers can open and withdraw in front of the block. The left hand stays clear. The right wrist withdraws in front of the stock, rolls the jaw opening into the horizontal plane, and advances both open fingertips against separated points on the near edge. The 64 mm opening provides two contacts from a single gripper. Near the lip, the fingertips aim just short of the lower stock before backing away. The waypoints use 588 ticks, leaving 12 ticks at the final target. This is one handling route. The operator may use either arm and may reposition either loose board.

Grip slip during the roll can change the landing position. Landing too far forward can trap the lower finger. A low pushing face can move the blue board, while a high one can pass over the ochre edge. The calculated sweep covers the stock. Continuous wrist reach, housing clearance, grip retention and the 0.5 mm push margin require checking with the installed gripper.

## Reading the finish

At the deadline, inspect the released pair. Follow the face finishes through the turn if the boards exchange places. Sight along the edges for a lifted corner. At each end of the operator-near long edge, read the lateral and forward offsets with a millimetre rule beside the stack. Their Euclidean distance must be at most 3 mm at each end. Include both near ends and the far ledge in the overview. For a borderline offset, use the common-datum caliper method below.

With the blue board underneath and both near endpoints exactly matched after a forward-axis roll, the far-right blue corner projects 12 mm. For a comparison, slide the unturned ochre board 12 mm left. The same far ledge appears, but both near endpoints now miss by 12 mm and the pale face is still up. Aligning just one endpoint also leaves the pair free to pivot. The two endpoint readings and the face change establish the comparison. The far ledge follows from the cut geometry.

The eight equal-valued scoring alternatives allow both stacking orders and cover which original long edge of each board ends nearest the operator. They pair the physical left and right endpoints after turnover. The grader measures corner offsets at mid-thickness relative to the other board. It allows 0.3° from opposing original face normals, corresponding to about 1.02 mm of differential opening over the diagonal. Each in-plane axis of the blue board must be within 1° of horizontal. The thresholds for a released, resting stack are at least 0.20 N of board-on-board reaction, at most 0.005 N from each arm on each board, at most 5 mm/s translation per board and at most 3°/s spin per board. A 40 g board weighs about 0.392 N. These engineering readings implement the binary endpoint.

Every scoring alternative requires the `deadline` milestone. It counts 600 samples over the full possible range of the face angle, so time advances through pickup, contact, failed placements and retries. The one-sample tier checks the actual pose, reaction, release and motion at the deadline. An earlier match cannot bank the award. The paired controls reverse the order of contact and release at sample 600. Other controls cover early edge contact followed by reseating, renewed sliding, all four edge pairings and the inclusive 3 mm boundary.

## Environment and evidence

Use the fixed dual-YAM PhysX runtime at 20 Hz with RGB and robot joints as observations. Actions are joint targets and normalized gripper width. The fixture dimensions, motion plan and numerical reading tolerances are engineering choices. The cases are synthetic scorer inputs. The reference uses the same 39 mm reset as the zero-handling control. The fixed overview is used to inspect the exposed ledge. The transparent 3 mm radius circle is a proposed bench aid for checking borderline offsets. A real bench repeatability study has not been performed.

```
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

Independent trials supply the authentic model video, measured scores, screenshot and full-score demonstration when required.
The overview grader checks both near ends, both far corners of the top board and the lower corner that protrudes laterally beyond the top board. The lower far corner tucked behind the upper stock is irrelevant. At each required vertex, inspect the projected vertex and two points 4 mm along its adjoining stock edges. An arm over any of the three points hides part of the reading. Pixels on the bench outside those edges do not affect the reading. The check uses the same fixed overview as the recording. Wrist and overhead RGB and robot joints are the policy inputs.

Select the nearer long edge separately for each board. Compare the table x coordinates of the two edge midpoints, at local (−50, +3) and (+50, −3) mm. This accounts for the skew under rotation. The arm can turn either board; follow the colored edge and face through continuous footage even if the lower board returns underneath. For a bench reading, take the overhead and grazing photographs at the deadline before touching the boards. In the grazing view check the actual seam for a lifted corner and confirm that the robot is clear. Hold a small square against the fixed front face of the bench block so that it supplies a common fore–aft datum for both layers. Use the same end of the block as the left–right datum, with the square turned ninety degrees. On each axis, hold the square's vertical blade just beside the corner at each board's mid-thickness. Follow the adjoining straight edge to the vertex when a higher board hides its tip in the overhead picture. Move the gauge only, using a rule for a clear gap or a digital caliper with light contact for a close call. Record the two signed differences by subtracting the coordinates of corresponding corners on the same datum. Compute the planar separation as the square root of the sum of the two squared differences. Compare each of the two distances to 3 mm, inclusive.

Leave the pair flat on the raised work block for the reading. The lower board must bear on the flat top. A stack beyond the block can hide the far ledge behind the block and does not complete the inspection. In simulation, the active lower board must carry at least 0.35 N directly through the block, with its center 0.5–2.5 mm above the top surface (nominally 1.5 mm for 3 mm stock). The height and the grazing seam show whether a reaction comes from the top or a brush against the block wall, and the unchanged inter-board bearing test verifies the two boards rest together.

Keep the stack undisturbed through all sightline, registration and seating readings. Use the continuous footage and face finish to identify the turned board. If the lower face cannot be identified from the deadline picture and footage, first finish and record all endpoint readings, then lift only the upper board straight up to identify the lower face. A photograph of the lifted state only identifies the face and supplies no geometry or clearance reading.

The task card’s transparent 3 mm radius circle is a visual aid for viewing the gap from above. For a borderline corner, use a caliper to read both endpoints at mid-thickness from the same fore–aft and left–right datums. Subtract the lower endpoint’s coordinate from the upper endpoint’s on each axis to obtain signed offsets Δx and Δy. Compute the planar distance as √(Δx² + Δy²) and apply the inclusive 3 mm limit separately at both near corners. The common datums avoid parallax from board thickness and oblique viewing. Check gauge zero and repeat from the same datums. If the estimated uncertainty is +/- 0.02 mm per signed difference, the planar reading can move by at most 0.029 mm. The paired projected offsets (-2.4, 1.7) and (-2.6, 1.7) mm give 2.94 and 3.11 mm, respectively, and remain on their original sides at that precision. These examples use an assumed gauge uncertainty. If the gauge uncertainty interval spans 3 mm, record the interval and repeat with a better-resolved gauge or setup before scoring. Apply the ordinary 3 mm cutoff to the resolved reading. The episode always runs to the final observation.

At each registered edge, the lower board steps 6 mm to one side by the time it reaches its opposite edge and the reversed upper board steps 6 mm the other way. The exposed lower corner is the negative local-y end on its original far edge when its original near edge is registered. It is the positive local-y end on its original near edge when the original far edge is registered. The 3 mm endpoint tolerance cannot swap these lateral extremes. Reversing which board is on top exchanges the lower-board test.

</details>


## Reproduce the measured trials

Install the pinned [YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux with an RTX GPU, then source its yam.env. From this task directory run the following commands. Seed 0, 20 Hz.

```sh
uv pip install inspect-robots==1.0.0 inspect-robots-yam==0.38.0 dreamlabs-client==1.0.14 numpy scipy pillow "imageio[ffmpeg]" pytest
uv pip install "inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339"
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-demo
python -m task_impl.run dream --task . --out /tmp/robocurve-model
```

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 237.98 s for the scripted run and 226.98 s for the model trial.
