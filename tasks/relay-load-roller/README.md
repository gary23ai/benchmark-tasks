# Roll a classroom block on loose rollers

Exemplar 33, Lesson 2 asks pupils how to keep a brick moving after it rolls past its rear supports. A completed relay makes the answer visible twice. The block overtakes the painted roller, then moves farther with that roller turning under its front half. The wooden load, pre-mounted start, dimensions, markings and timing are choices for the tabletop exercise. [Primary School Curriculum, Science Teacher Guidelines, PDF page 120](https://www.curriculumonline.ie/getmedia/8afff1e2-e969-4f64-af35-937da5b0ad18/PSEC03c_Science_Guidelines.pdf#page=120)

## Station and reset

Use the calibrated dual YAM/PhysX tabletop environment with a 20 Hz control rate, normal joint targets and normalized gripper width. The robot observes RGB and joint state. World X points forward, Y points left and Z is height above the simulator table. In millimetres, the bases are at (0, +305, 2.5) and (0, −305, 2.5). Viewed from negative Y with Z up, increasing X appears to the right. Use the fixed reset and seed 0 wherever the runner accepts a seed.

| Object | Material and dimensions | Mass | World center at reset, mm |
|---|---|---:|---|
| Fixed desktop insert | Hard maple, 410 × 245 × 12 mm | 840 g | (346, 18, 6) |
| Block | Uniform beech, 140 × 80 × 35 mm | 275 g | (284, 18, 45.5) |
| Marked rear roller | Solid hardwood, Ø16 × 140 mm | 19.7 g | (224, 18, 20) |
| Middle roller | Solid hardwood, Ø16 × 140 mm | 19.7 g | (292, 18, 20) |
| Front roller | Solid hardwood, Ø16 × 140 mm | 19.7 g | (329, 18, 20) |

Mount the insert level. Its working surface is at world Z = 12 mm. Lane station zero is at world (214, 18, 12) mm. Stations increase along X. The start line is centered at zero, the pale finish band spans 130–150 mm, and its dark tick marks 140 mm. These are painted marks outside the roller sweep. Keep stations −55 through 315 and lateral offsets of ±95 mm clear of obstacles.

Set the roller axes parallel to Y at stations 10, 78 and 115 mm. Match the physical diameters within 0.2 mm. Square both ends within 1 mm of nominal and turn the marked ends dark-half up. Four adjoining paint strips approximate a half-disc on each end. Place the block with its trailing face at zero and its underside 16 mm above the insert. Each roller projects 30 mm on both sides. Remove the ruler and square, clear both hands and start at rest. Every loose body is free. The insert has friction coefficient 0.58 and the loose wood parts use 0.55.

## Manipulation and feasibility

The scripted path first brings the left hand above the exposed ends and the right hand behind the block. The right fingers lightly grip across the trailing end at constant height to move and slow the block. The first nominal push is lengthened 24 mm to unload the rear roller before the pickup. Verify complete clearance before lifting. With rolling contact at both surfaces, the remaining axes would then be at 100 and 137 mm, on either side of the block midpoint at 114 mm. The freed roller can coast. The pickup target uses the measured released position and approaches the exposed end slightly inboard with a shallow lean toward the roller.

The left fingers lift the roller end to world Z = 94 mm. For a level roller this leaves 23 mm between its underside and the top of the block. The hand crosses above the block, lowers the roller at station 200, releases and withdraws. The right hand resumes the push and releases at the band. This is one candidate path. The operator may use either hand, pause elsewhere and use another grasp. At 140 mm of ideal travel the marked axis would be near station 240, the original front axis near 185 and the block midpoint at 210.

Check the installed fingertip clearance at the low pickup, far-end clearance during the sweep, arm separation and coast after release. A skewed or slightly high roller may be pushed ahead by the square leading edge. Load sharing between three nominally equal cylinders is sensitive to contact margins. These effects can determine whether the proposed path completes within 600 ticks (30 seconds).

## Reading the attempt

Use continuous side footage along the roller axes. Calibrate with a ruler in the plane of the near block face, then remove it. Keep the desk gap, both lower block corners, the painted end, both hands and finish marks visible. At a 360 mm horizontal field, the 16 mm end occupies about 14 pixels of a 320-pixel image. An obscured transfer or unreadable band makes the physical reading indeterminate.

Follow the painted end from its initial loaded turn until the block passes it. Complete clearance places the roller center at least one radius behind the trailing face while the trailing face itself has advanced at least 27 mm. For transfer credit, the roller must then be released on the insert with its center at least one radius ahead of the leading face, while both remaining rollers bear the block. The instrumented reading uses at least 0.08 N on each remaining support and at least 0.08 N between the marked roller and insert. Crosswise placement allows 32 mm of lateral center offset, 16 mm of fore–aft difference between the roller ends and 3 mm of end-height difference. Before clearance, raising the marked axis to 12 mm above the insert or displacing its center sideways by 20 mm triggers premature removal. A touch without that displacement permits recovery.

For onward rolling, watch the block edge advance and the painted end move and turn beneath it. The instrumented grader takes two ordered six-sample readings with the trailing face first at or before 100 mm, then at or beyond 125 mm. Each requires at least 0.08 N between block and marked roller, at least 0.15 N between roller and insert, and released hands on the marked roller. On each six-sample run, measure at least 1.25 mm of forward block travel and 0.75 mm of forward roller travel. Follow the visible paint boundary and require at least 10 degrees of forward turn. A missing support sample starts the window over. Watching the two displacements and the signed turn distinguishes a loaded advance from repeated rocking or moving backwards.

For full credit, read the final second, samples 581–600 inclusive. The trailing-face center must stay within the 130–150 mm band, underside height within 13–19 mm, tilt within 3 degrees and heading within 8 degrees. The marked roller bears at least 0.08 N under the front half. Either original roller may supply the rear support, also at least 0.08 N. Each support center must be at least 1 mm from the block midpoint on its respective side. The crosswise placement tolerances still apply. Rest allows up to 3 mm/s for both the block and the marked roller, and 12 degrees/s for the marked roller. The heading and tilt bounds apply throughout the final second. Hands clear allows up to 0.02 N from each arm on each loose component.

The scorer counts the released rest over the final window. Every award also requires a 600-sample clock. At that reading the full tier checks the finish geometry, support, rest and all eight arm-to-object contacts again. A hand returning on sample 600 leaves only the earned transfer credit if the block still rests level on rollers. Partial credit also checks level support and desk clearance on sample 600. An earlier transfer cannot preserve credit for a block left on the desktop.

Carrying the block or dragging it on the insert overrides an earned transfer. The instrumented carry checks use underside height of at least 20 mm with all roller contacts at or below 0.02 N, or hand-borne motion of at least 6 mm/s with arm contact of at least 0.15 N and all support contacts at or below 0.02 N. The drag indicator uses at least 0.25 N of insert contact with block speed of at least 8 mm/s. Review impacts and very slow slides against the footage when assessing these thresholds.

## Controls and independent evidence

From the same reset, push slowly while leaving the rear roller behind. With a centered block and ideal rolling, the midpoint overtakes the original front roller after 90 mm. At the near finish boundary it would be 20 mm beyond that support. Observe where the nose actually drops on release. For a matched endpoint, prop the block near the finish and insert the marked roller before releasing. The still image can match the requested finish. Continuous footage shows whether the block ever traveled onward on that support.

The declarative controls include an early touch followed by a valid retry, premature removal, carrying, a late insertion and both finish-band boundaries. Three timing controls distinguish a hand clear for all of samples 581–600, contact on 599 followed by withdrawal, and a hand returning only on 600. A separate control loses level support on the last sample after earning the transfer.

## Commands

From the single task folder in the trusted environment, use the following commands.

```
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

In the recorded reference, the final trailing-edge position stayed between 138.58 and 138.63 mm. The returned roller and the original front roller supported the block throughout the final second. The construction and camera instructions describe a method for physical replication.

## Recorded evaluation

Dream-1 run: 0 (600 steps); demo: 1.0 (600 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 260.32 seconds for the demo and 226.84 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

The reference video is from the earlier native run with the identical scene and scripted controller. Its recorded measurements were replayed against the submitted scoring rules. The old scoring code returned 0.5. The submitted scorer returns 1 on that recording. The Dream-1 trial uses the submitted source.
