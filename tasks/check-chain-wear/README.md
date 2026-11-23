# Check a stationary bicycle chain with a wear gauge

The mechanic needs a reading that can be inspected after release. Park Tool’s Method 4 places the reference hook in an inner-plate link. Figure 1 of the gauge instructions shows full tip entry. Figure 2 shows the roller stopping the tip high. The chain manufacturer’s replacement guidance determines how to act on the reading. [Gauge instructions](https://www.parktool.com/assets/doc/product/CC-3.2_instructions.pdf) · [Park Tool repair guide, Method 4](https://www.parktool.com/en-us/blog/repair-help/when-to-replace-a-chain-on-a-bicycle)

## Station and reset

Dimensions here are in millimetres. Table coordinates are `(x forward, y left, z up)`. The YAM bases are at `(0, 305, 2.5)` and `(0, −305, 2.5)`. All bodies start unrotated. The deterministic layout uses seed 0 when the host offers a seed setting.

| Object | Construction and reset position |
|---|---|
| Stand and end supports | Fixed 8 kg steel assembly, frame at `(290, −50, 120)`. A 45 × 180 × 12 foot and 25 × 25 × 145 post support a 12 × 190 × 12 rail 95 behind the chain. Two 102 × 6 × 8 arms end in paired 0.8 × 18 × 8.2 continuation plates. The two end pins represent the stationary drivetrain beyond the exposed section. |
| Chain plates | Twelve steel links. Roller `k` is at `(290, −127.1 + 12.85k, 120)` for `k = 0…12`. Link `k` is halfway between rollers `k` and `k+1`. Even links have inner plates 0.8 thick and 8.2 high, with 2.38 between their faces. Link 0 has round ends of radius 4.1 on 12.85 centers. The other inner plates have 19.6-long rectangular envelopes. Odd links have 0.8 × 19 × 7.4 outer plates with a 4.18 gap and weigh 2.0 g each. Inner links weigh 1.8 g for links 0 and 10, 2.5 g for link 2, and 3.2 g for links 4, 6 and 8, including any integral rollers. |
| Rollers and pins | Each steel roller is 7.72 in diameter and 2.32 wide along x. Rollers 0, 1, 2, 10, 11 and 12 are individual 0.65 g bodies with passive transverse bearings. The remaining seven are integral to their inner links. Adjacent links articulate about passive transverse pin joints. The end pins constrain the span to twelve extended pitches. |
| Gauge | A 35 g stainless steel body with a 141 × 19.2 × 1.5 blade. Its frame starts at `(220, −63.1, 65)`. Two angled faces locate the reference roller and a 1.6-wide stem enters the inner link. The measuring tooth is 2.45 wide along the chain and ends in a 1.85-wide nose. The 8-wide shoulder above the tooth bears on the remote roller. A blue mark on both blade faces identifies this end. |
| Cradle | Fixed 110 g acetal rest, frame at `(220, −63.1, 0)`. A 36 × 104 × 6 base carries two 12 × 9 saddles 84 apart. The bearing surfaces are 71.4 above the table, with 2.2-wide slots and 4-high lips. The center grip has at least 25 clear on either face. |

On the physical station, place the work surface beside the bicycle with the crank and wheel clear. Select a compatible worn eight-speed chain on which the same gauge seats under its own weight. Record the stand setting, rear sprocket, front chainring, crank position and link count from the master link to roller 0. Restore those settings before each trial. The count identifies the specimen even if the cranks have made a full turn. Put the blade in both saddles with the marked end toward +y. In simulation the declared positions and pin anchors reproduce the section directly.

## Placement and clearances

The script uses the right gripper at the center of the blade with its fingers down and its closing direction along x. The pickup tip is at `(220, −63.1, 79)`, 14 above the gauge datum. It lifts vertically, crosses at tip height 155 and lowers at `(290, −63.1)`. The final closed-jaw target and the opening target coincide at `(290, −63.1, 134.3)`. Both are fixed to the stand. Their y coordinate is 64 from the pinned center of roller 0, so spinning that roller cannot rotate the commanded offset. The jaws open before the vertical retreat. The trajectory allocates 690 control steps to pickup, placement and withdrawal, leaving 210 steps at the last target. The operator may use either hand and either fitting heading.

The station placement reduces the horizontal distance from the right base to the release point to about 378 mm. The following clearances come from the specified geometry and nominal grasp. During the crossing, the bottom of the measuring tooth is at z = 138, which clears the highest plate by 13.9. At the opening target, the hook’s roller datum is at z = 120.3, leaving 0.3 for gravity to close. The fingertip target is 10.2 above the highest plate. The 24-wide opening moves the inner jaw faces to nominal offsets of ±12 from the chain plane. The nearest rail face is 89 behind that plane, and the cradle saddles begin 37.5 to either side of the central grasp along y. These dimensions describe target positions. Actual joint interpolation, outer finger geometry, blade slip and release error require a swept-motion check on the calibrated arms.

## Why the roller can block the tooth

The modeled working edge spans ten nominal pitches. A new-chain pitch of 12.70 gives 127.00 over ten pitches. At 0.75% elongation that span is 127.9525. From the hook datum to the far face of the tooth is 124.0925. Adding the 3.86 roller radius gives 127.9525. The modeled worn span of 128.50 therefore leaves 0.5475 of clearance at roller-center height. A new chain gives 0.9525 of interference there. The hook has to establish the reference for either prediction to hold.

The geometry represents one working edge. It approximates the hook with planar bearing faces, the inactive links with compound bodies and the rest of the drivetrain with end supports. Pin compliance, contact margins and the shape of a particular commercial tool can affect the go/no-go boundary.

## Inspecting the reading

Look down the plate edges at both working ends. The inner plates must flank the hook stem, and the remote plates must flank the measuring tooth. From the side, compare full entry and shoulder seating with Figure 1. Trace the near plate past the tooth if the gauge appears level. A tooth wholly beside the chain can give the same side silhouette. Also inspect the shoulder. At the remote roller, the adjacent inner plate rises 0.24 above the roller crown. A tooth can enter the wider opening while its shoulder catches there.

Award 1 for full entry with both roller bearings established and the gauge released and stable throughout the final 0.5 s. Use the final footage to check continuity. An untouched gauge, a blocked or outside tooth, a shoulder resting on a plate, and hand support during that interval each earn 0. An earlier contact error is recoverable. On the separate new-chain control, a correctly referenced tooth stopped high is the expected negative wear reading.

The simulator accepts hook-to-remote pairs 0→10, 2→12 and 11→1. Relative to the selected hook roller center, the reference datum may differ by ±0.45 along the run and ±0.5 vertically. The tip bottom must be 5.1–6.2 inward from the remote roller center and 2.5–3.5 below it. The hook stem and measuring tip centerlines must be within 0.44 and 1.34 of the chain plane respectively. Those allowances subtract half the blade thickness from half the corresponding plate gap.

Each selected roller must exert at least 0.012 N on the gauge. Gauge contact from either arm may be at most 0.002 N per arm. Each of the ten readings must keep the gauge within 4° of upright, maintain both roller contacts and keep the tooth fitted between the plates. Translational speed must remain at or below 2 mm/s. Bounds are inclusive. One pair must satisfy every condition on samples 891 through 900. Its tier also checks the live measurements on sample 900. The cases exercise a retry, a release one sample late, support interrupted before the last frame, and loss of either roller on the last frame.

## Environment and evidence

Use one `check-chain-wear` task folder containing the decoded fixture, cases, English card and README. The trusted dual-YAM PhysX environment runs at 20 Hz with RGB and robot-joint observations. The actuator uses ordinary joint targets and gripper width. The 900-step limit is 45 seconds.

```
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

At the independent evidence gate, measure calibrated fingertip position and orientation through lowering and opening, including minimum swept clearance. Compare gravity seating on the selected worn section with the same lowering motion on a compatible new chain. For the simulator control, use pitch 12.70, keep roller 0 pinned, and derive every link center, pin anchor, roller center and far support from that pitch. Keep the gauge geometry, reference placement and release height identical.

Compare the released seat, high tooth, outside tooth and shoulder caught on the plate in the stock 320 × 176 view. An oblique view spanning about 220 mm is a starting point for that inspection. These instructions describe checks for a physical reproduction with a commercial tool. The accompanying simulator footage shows the fixed worn-chain start and the released gauge on the rollers.

The instrumented judge measures the physical tooth corners in the matching link opening. It permits boundary contact and checks both headings. Qualify the actual edge measurement with a close view of a fitted tooth and one contacting the plate from outside.

The pickup closes farther onto the gauge spine, raises it a short distance, then finishes the lift before transporting it to the chain.

The example preserves one continuous grip command during the lift, traverse and gauge seating.

## Recorded evaluation

Dream-1 run: 0 (900 steps); demo: 1.0 (900 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 324.87 seconds for the demo and 296.76 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

The reference video is from the earlier native run with the identical scene and scripted controller. Its recorded measurements were replayed against the submitted scoring rules. The old scoring code returned 0. The submitted scorer returns 1 on that recording. The Dream-1 trial uses the submitted source.
