# Repair the raised joint in a preschool marble run

The receiving track already points at the tray. Its raised inlet blocks the marble. The visible result of a repair is the original marble crossing the join under gravity and rolling along the loose track after the operator lets go.

The Iowa Regents Center documents this placement error, a child's test and another child's correction in “Communicating Effectively.” Its separate “Fine Motor & Spatial Reasoning” observation describes aligning sections and positioning supports. These classroom observations support the category Other through early-childhood construction play. The square channels, dimensions, mounts, gate and tray are station design choices. [Classroom observations](https://regentsctr.uni.edu/ramps-and-pathways-media) · [Ramps & Pathways curriculum](https://regentsctr.uni.edu/ramps-pathways)

## Station drawing

All drawing dimensions are in millimetres. The origin is on the tabletop beneath the feeder outlet, at table coordinates (265, −112, 0). Positive x follows the run, positive y points left and positive z points up. Both tracks start centered with zero roll and yaw. The YAM bases are at table coordinates (0, 305, 2.5) and (0, −305, 2.5).

| Part | Construction and fixed start |
|---|---|
| Feeder | Mounted maple channel and pedestal, 140 g together. The 160 × 32 floor runs from x = −160 to 0. Its upper face falls from z = 48 to 40 and its underside is 6 lower. The 3-wide rails rise 8 above the floor, leaving 26 clear between them. The 30-wide pedestal ends at x = −35. The last 35 is cantilevered and the outlet is plumb. |
| Receiver | Loose maple channel, 160 × 46 outside. The 6-thick floor weighs 28.7 g. The two bonded sidewalls weigh 6.7 g together. Each is 4 wide and rises 8 above the floor, leaving a 38-wide opening. Its inlet bottom midpoint starts at (−15, 0, 52.225) and the channel pitches downhill 15.72°. A 20-long stripe on the −y exterior gives the travel scale. |
| Supports | Two fixed beech blocks, 12 × 50 and 11 × 50 in plan. The near block weighs 8.3 g and its transverse crest is at (x, z) = (20, 22). The far block weighs 3.5 g and its crest is at (135, 10). The upstream faces fall 2.7 mm over 6 mm. The near block has the same downstream face. The far block falls 2.25 mm over its 5 mm downstream face and ends at x = 140 mm, against the tray edge without overlapping it. |
| Tray | Fixed rigid plastic, 51 g, 93 × 76 outside. The floor is 2 thick. A 3-long lip starts at x = 140 and reaches z = 5. The inner side faces are at y = ±35 and the inner back face at x = 230. The three walls are 3 thick and rise 35 above the floor. |
| Gate | Moving acetal assembly, 14 g. A 3 × 24 × 22 vertical blade has its upstream face at x = −80. Its lower edge travels from z = 45 to 73. A 9 × 78 × 3 bridge carries a 16 × 10 × 12 tab centered at (−78.5, 36, 76) when closed. An 8 × 8 × 63 stem runs in the outboard guide. |
| Guide | Mounted acetal sleeve, 12 g, centered at (−78.5, 62) in plan. An 18 × 18 × 3 base carries a 44-high square sleeve with a 12-square bore. The passive vertical slide has 28 travel. Gravity closes the gate. |
| Marble | Orange-painted hardwood sphere, 2.8 g, diameter 20. Its center starts at (−90, 0, 54.513), supported on the feeder against the upstream face of the blade. |

## Reset and manipulation

Start both arms at the stock YAM home (0, 1, 1, −0.6, 0, 0) radians with both jaws fully open. Empty the tray and close the unloaded gate. A removable gauge sets the inlet position and centerline. The receiver rests on the two feeder rim tips and the far crest, with about 20 mm of clearance above the near crest. Withdraw the gauge, check that the receiver stays put, then load the marble.

The first waypoint parks the open fingertips at coordinates measured from the workboard (186.5, −17, 130) and (328, −207, 130). The example trajectory grips the receiver across its exposed sides near midspan, lifts it 20 mm and translates it 23 mm downstream. With the inlet clear of the feeder, it reduces the pitch and lowers the receiver onto the two crests. Their 12 mm drop over 115 mm gives a downhill pitch of 5.96°. The nominal inlet ends about 4 mm beyond the feeder outlet. The jaws open and withdraw before the other hand lifts the gate by its tab. The path allocates 616 control steps to the waypoints and holds the final gate target for the remaining 384. A nested join or another placement that carries the marble unaided also qualifies.

## Reading the trial

Use one continuous elevated oblique view from −y. Include the loaded gate, hands, joint, receiving surface and tray. A 400 mm horizontal field spans roughly 16 pixels per marble diameter at 320 × 176 before foreshortening. Confirm that the near rim and the gate hand leave the landing visible. The real tray can use a clear near wall. The simulated tray has opaque walls, so camera elevation matters. Repeat an obscured trial from reset.

Follow the same ball from the feeder onto the receiver. Give 0.5 for a continuous supported segment advancing at least 20 mm downstream. Measure from the ball center projected onto the receiving ruler at its first supported reading. After a short landing bounce, start that measurement again at the first floor-supported position on the receiver. A numbered 20 mm stripe follows the track if it moves. Give 1 if the ball also stays on the tray for the final second and its largest three-dimensional displacement between any two of the last twenty readings is at most 2 mm. A blocked ball, a rim strike without a roll, or a direct drop into the tray earns zero. Either hand touching the marble at any time, holding a track at release, or handling one afterward overrides all credit through the final reading. Holding the raised gate is permitted.

## Simulator readings

First record the loaded ball at rest on the feeder with the gate closed. A 2 mm upward excursion recognizes the opening. Check track clearance at the onset and throughout that excursion. A smaller movement returning within 0.4 mm of the closed extreme can re-arm a clean attempt. After opening, require feeder support downstream of the blade, then one continuous, directionally advancing supported segment inside the receiver. Tray contact away from the receiver before the supported crossing disqualifies a direct catch. After gate release and before the qualifying receiver roll, contact with either fixed support crest or the workboard disqualifies the handoff. A bounce on the receiving floor can start a fresh qualifying segment. The fixture rests on a clamped 580 × 360 × 6 mm sealed workboard. Positions in the fixture drawing are relative to its upper surface. The simulator checks measured normal reactions. A hand-to-marble signal above numerical zero (10 nN) on any scored reading gives zero. Visible ball contact is forbidden, including after the ball reaches the tray. The block and workboard checks end on the sample that qualifies the 20 mm supported advance. That sample must itself carry the marble on the receiver floor without bearing on a block or the workboard. A subsequent tray miss retains the supported-roll credit. A free-flight crossing or a bounce on the receiving floor can still advance after the marble regains support. Use the continuous close frame to distinguish the underlying wood crest, table and receiver.

Use at least 1 mN of measured floor contact and the loaded receiver bounds, never sidewall contact in place of bearing. Measure signed travel along the actual running floor. Its forward axis is (0.9626, 0, −0.27094) in the bed body frame. Rotate that direction with the bed at each sample. Measure from the first supported ball center, then begin a new segment after a hop. Latch the roll at 20 mm of net downstream travel. A flight or leaving the supported corridor starts a new segment. A backtrack while the marble remains supported counts against the net advance from the original position. The simulator also retains the fixed receiver-relative lateral, station and height bands to distinguish a wall glance from floor travel. Body-to-body contact channels with the near and far crests are diagnostics for the support surfaces.

Use a fixed close side view covering the stripe, running floor and ball during the receiving segment. If the near wall screens the gap, add a synchronized view from the open tray end. Compare a supported roll with a hop ending in the same apparent place. After a hop start the measurement at the first supported point. For the tray, center an overhead field of no more than 110 mm and put the ruler at the seated ball center height. Use a synchronized oblique view to establish contact with the floor. Compare calibrated 1.8 and 2.2 mm diagonal out-and-back paths over the same twenty readings. If uncertainty crosses the decision boundary, repeat from reset. The passive motion of the receiver after an unaided roll does not cancel earned credit.

Full credit requires 20 consecutive final samples with at least 1 mN tray contact, greatest pairwise center span at most 2 mm, and ball center inside x = 143–220.8 mm, |y| ≤ 25.8 mm, z = 11–13 mm in the drawing frame.

A measured arm reaction of at least 10 nN on the marble gives zero on any scored reading. This is the simulation cutoff for numerical zero. On a track, the forbidden-contact cutoff is 2 mN. For each arm, sum its receiver floor and sidewall reactions on the same sample. The earlier forbidden contact remains disqualifying after withdrawal.

## Physical qualification

Open only the gate from the dimensioned reset, leaving the receiver free to shift. The perched inlet leaves about 11 mm beneath it for a 20 mm ball. The descent from the gate supplies too little energy to climb the raised floor if the receiver stays perched. A gate-only passage would show that impact can defeat the intended fault. From a fresh identical reset, the mounted-YAM repair must produce a visible unaided passage under the same release. Check grip slip, clearance during withdrawal, rebound off the tray lip and the final sightline. Physical feasibility within 50 seconds and the force tolerances await independent trials.

## Environment and commands

Use one `repair-marble-run` folder with the fixed dual-YAM/PhysX runner, rigid tabletop and gravity. Reset geometry is deterministic. Use seed 0 where supported. The policy receives RGB and joint observations and commands ordinary joint targets and normalized gripper width. Run from the task folder.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

The subsequent evidence gate supplies authentic trial footage, any required real full-score demonstration, measured scores and the screenshot.
At the full episode horizon, score only the final recorded result. The runtime continues observing after a temporary high score.


The scripted repair holds its commanded grip between the successive lift and track-alignment positions. The fixture and the scoring criteria are the same throughout the trial.

## Recorded evaluation

Dream-1 run: 0 (1000 steps); demo: 1.0 (1000 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 327.87 seconds for the demo and 301.99 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```
