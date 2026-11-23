# Mount a bowl on the side pins of a bowl-lift mixer

Mount the free, empty bowl on the lowered cradle and press the rear stud into the spring clip. A perched stud can leave a bowl apparently level until the operator lets go. KitchenAid documents placing the bowl tabs over both pins and pressing the back into the latch. The manufacturer's 3-quart KN3CW bowl-and-whip product envelope is about 279 mm wide, 229 mm deep and 121 mm tall. This fixture has a nominal 226 mm rim, 115 mm height and approximately 285 mm overall span across the mounting tabs. Its working size is comparable. The passive rear clip is visible through two open sides of its housing. The motor head remains fitted throughout installation. [Assembly procedure](https://producthelp.kitchenaid.com/Countertop_Appliances/Stand_Mixers/Product_Info/Stand_Mixer_Assembly_and_Operation/Assemble_and_Adjust_a_Bowl-Lift_Mixer) · [KitchenAid accessory dimensions](https://images.thdstatic.com/catalog/pdfImages/42/42958631-1cc3-4e5c-ab5f-cd31ca9f49a4.pdf)

## Apparatus and reset

All distances are in millimetres. Positive x points from the operator toward the rear column and positive y toward the left arm. Bolt the lowered cradle to the bench at (360, 0, 128), the midpoint between the locating pins at shoulder height. Pin spacing is 246.4. The steel shank diameter is 6.4. Each 4.8-thick stainless tab has a 10.4-across-flats closed octagonal opening. The unloaded bowl weighs 0.60 kg and its foot is 80 below the tab datum. The orange spring-loaded rear clip retracts up to 12.8. The nominal spring preload is 2.56 N. The 12.8-diameter rear stud has blue end markers visible from both sides. All fixture geometry, dimensions, friction, density and mating offsets are specified in task_impl/fixture.json. The model is an original reconstruction. Arm and slider forces are estimates.

The fixed die-cast machine has a 350 mm deep bench footprint and reaches 409 mm above the bench at the steel drive housing. Its red motor casing, gearbox, planetary tool shaft, rear upright and lowered lift handle have physical collision geometry. The lower column relief leaves more than 7 mm of clearance behind the rear stop even at full passive slider travel. The beater is removed, as it may be when installing an empty bowl. The bowl is empty inside, with the mounting ears, stud and handle welded to the exterior. Its entire payload is its 0.60 kg shell and attached fittings. The bowl uses 24 circumferential wall sectors with four tapered bands, with identical visible and collision outlines.

Set down the empty bowl at (184,0,80) on its foot with the front handle facing the operator. The tabs are horizontal and 176 ahead of the pin line. Mark the two pin tops yellow, both ends of the rear stud blue and the moving lip orange. Reset the slider to its spring-returned forward stop. After each successful run, support the bowl, retract the spring-loaded slider and lift the rear stud and both ears free. Return the bowl upright to its mark. Keep the operating fixture unpowered.

## Scoring at the bench

Score the final uninterrupted second, or finish early when the 1.0 state lasts a second. Use synchronized views of both yellow pin tops, the left and right rear pocket windows, and both hands. Score 0.5 if each yellow pin passes through its own closed tab and projects 5–9 mm above the top face of the tab (7.6 mm with the tab resting on the shoulder). The tab top lies between the two black height rings on the pin. Use the 6.4 mm steel shank for scale. In front or overhead view, each gripper centre (the midpoint between its jaws) must be at least two shank diameters outboard of the outer edge of the corresponding tab. Both arms must be visibly clear of the bowl and catch. Score 1 if during the same second both blue stud ends sit entirely below and behind the orange lip in the respective pocket windows, and the collar stays within the 3 mm mark at its forward stop. The stud centre must be 5–15 mm above the pocket floor and 5–15 mm behind its front face at each window. A stud on the ramp or a retracted catch caps the score at 0.5. One missed tab or continued hand support earns zero. A pin touching the inside wall of a closed ear and a blue stud touching the underside of the lip both count when their visible positions meet these marks. A failed attempt may be retried.

Two black rings on each shank mark the accepted tab-top heights. Scribed marks on the sill and front jamb bound the accepted stud centre, and black witness lines mark zero and 3 mm beside the moving collar. Align the overhead camera to show each coloured tip emerging inside its closed ear and the two withdrawn grippers. The side cameras see the blue disks through the two inspection windows. Use the 12.8 mm stud and 10 mm spacing of the two sill ticks as scales. The near edge of a disk must extend under the lip and its outer face must extend past the corresponding lateral lip edge. A hand covering a required measurement cannot verify the missing condition. When hand motion makes a frame ambiguous, the second of uninterrupted visible success must start afresh.

The native grader checks physical pin and bore geometry at both tab faces. It compares the marked stud endpoints and collar with the same pocket and stop coordinates as the bench test. Whole-arm contact signals reject continued external support. No positive shoulder force or impulse threshold contributes to the score. After mounting the ears, the reference clears both grippers and pauses to expose the side-pin seating. It then places the two grippers on the rear rim and presses it down, then withdraws. The reference controls the original robot joint and jaw inputs at 20 Hz. Each trial allows 830 actions. Actor transforms are assigned at reset only.

## Reading the recorded demonstration

From 26.9 to 29.5 seconds both hands are clear. Each yellow locating pin projects through its own ear, while both blue stud ends are still above the orange lip. This earns 0.5. At 34.4 seconds the rear rim is being pressed and the passive catch has moved back 7.72 mm. In the last second the yellow tips still project through the ears, the blue ends are underneath the returned lip in both side windows, and the two hands are clear. This earns 1.0.

The last 20 measurements give exposed pin heights of 7.69 mm on the left and 7.83 mm on the right. The blue centres are 10.41 and 10.40 mm above the pocket floor and about 6.40 mm behind its front face. These measurements correspond to the marked boundaries in the video. Both hands are free of contact with the bowl and the clip throughout that second.

## Environment

Use the pinned dual-YAM tabletop simulator and run the tests from this task directory. All environment assets in this folder are original procedural geometry. The machine is unplugged. The original fixed motor housing, front hub, planetary drive and two lowered support arms bound the robot workspace. No powered mixing is simulated.

The referee recording also includes independent fixed RTX cameras aligned with each rear pocket window and the two pin shanks. The original overhead and wrist frames remain available to inspect hand withdrawal. These extra observations are excluded from robot policy input. A real referee may place ordinary close-up cameras at the same locations or observe those sides of the mixer directly.

For the recorded bench views, four additional Isaac Lab fixed RGB sensors are registered before the station is built. Each frame preserves its native camera pixels and optical occlusion. From a side of the machine, positive y denotes the operator’s left and the reference plane is the bench surface. In metres the rear cameras are at (0.487, ±0.155, 0.139), pointed at (0.486, 0, 0.132). Each pin camera is at (0.321, ±0.260, 0.169) and points to the pin on its own side at (0.360, ±0.1232, 0.135).

During the carry the reference angles its wrist housings outboard of the fitted motor case. Aggregate contacts with the static housing are logged to identify interference.

The reference closes toward a 23 mm gap on the 32 mm outside width of each ear. Four independent native pad-to-bowl force channels are logged during manipulation. These channels audit reference load and are excluded from scoring.

An additional wide three-quarter camera records the complete appliance and its workspace from the right rear, clear of the parked robot wrist. The setup screenshot combines the overhead and three-quarter images from the same reset state. The monitoring cameras never provide policy inputs.

KitchenAid lists 240185 as the bowl-retaining pin. Its replacement is WP240185. The genuine part has a narrowed pilot nose, a cylindrical shoulder and a separate press-fit mounting section. The modeled 6.4 mm shank and 12.4 mm exposed height are approximate proportions read from the part photograph with a one-inch grid. Overlapping circular collision shapes form its tapered nose. The grader checks the cylindrical bearing section through both faces of each ear. [Manufacturer parts list](https://www.kitchenaid.com/content/dam/global/documents/202305/repair-parts-list-w11678562-reva.pdf) · [OEM pin on a one-inch grid](https://www.kitchenaidparts.com/PartDetail/Pin/WP240185/1376925) · [OEM tip profile](https://www.searspartsdirect.com/product/3ngdmgahn3-0011-665/id-wp240185)

The rear stud is welded 126 mm behind the ear axis, beyond the exterior wall of the bowl. The window and spring guide are on the same side of the wall as the stud. The bearing faces on the lowered lift arms are 128 mm above the unpadded bench. Bowl-to-fixture contacts are recorded for diagnosis and do not contribute points.

The black rings and pocket marks carry the visual decision. A surface in contact may lie on its boundary. The numeric verifier allows 0.2 mm of collision-solver tolerance at the pin wall and 0.3 mm at the lip underside. The end of the blue disk and yellow-tipped pin must still be visible on the correct side, and the 5–9 mm pin-exposure and 5–15 mm rear-window intervals still apply.

## Recorded evaluation

Dream-1 run: 0 (830 steps); demo: 1.0 (805 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 342.88 seconds for the demo and 319.33 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```
