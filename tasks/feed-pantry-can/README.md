# Restock a gravity-feed pantry rack

The operator loads fresh stock at the upper mouth. Gravity takes the yellow tin to the back of the shelf and returns it to the pickup queue. Blue, the older tin, stays available first. [Shelf Reliance operating FAQ](https://www.shelfreliance.com/pages/faqs)

Home food-storage guidance recommends putting older cans within reach at the front when restocking. On a two-level rack, a correctly loaded can rolls behind the older can without requiring the household to rearrange the lower shelf. A can left stranded on the upper rails interrupts that rotation. [Utah Division of Emergency Management, household food rotation](https://dem.utah.gov/food-and-nutrition/)

## Environment and fixed layout

Use one task folder named `feed-pantry-can`. Use seed 0 and the standard dual-YAM home configurations in the trusted PhysX tabletop environment. The policy receives RGB and robot joints and commands joint targets and normalized gripper widths. The trial limit is 600 samples at 20 Hz, or 30 seconds.

Dimensions below are in millimetres. World x increases into the shelf, y to the operator’s left and z upward from the level tabletop. The left and right arm bases are at (0, +305, 2.5) and (0, −305, 2.5). The rack datum is world (360, 0, 0), on the shelf directly below the center of the upper mouth. The pickup opening faces the arms. All actor frames start unrotated. The rails and lip are fixed to the rack, which stands on its feet.

| Object | Dimensions, material and mass | Reset placement |
|---|---|---|
| Side frames, feet and rear guide | Molded polymer, 0.808 kg including the guide strips and their extensions. Two 45 × 142 × 8 feet, 332 mm side bases, 8 mm thick side ribs and 116.4 mm clear inside width. Rear guide 8 × 116.4 × 140. Maximum height 218. | Feet centered 22 and 294 behind the datum. Inside face of rear guide 318 behind it. |
| Paired upper ledges | Molded polymer, 0.065 kg together. Each ledge is 227.7 × 11.4 × 10. | Ledge centerlines at y = ±52.5. From the mouth, the bearing surfaces descend rearward at 7° from about 190 to 162 above the shelf. |
| Two retrieval ledges with rear landings | Molded polymer, 0.044 kg per side. Each has a 257.92 × 11.4 × 10 rail and a 62.89 × 11.4 × 10 landing. | Centerlines at y = ±52.5. The surface starts about 24 above the shelf at the pickup end and rises rearward at 7°. The 25° landing extends from about 256 to 313 behind the mouth. |
| Pickup lip | Molded polymer, 0.025 kg, 8 × 116.4 × 38. | Center at world (360, 0, 61). Blue rests against its rear face. |
| Blue and yellow tins | Closed dry steel tins with secured concentric ballast, 0.280 kg each. Each 71 mm diameter barrel is 106 long. Two 2 mm thick, 73 mm diameter end seams bring overall length to 110. | Blue center at world (400.5, 0, 65.75) with its axis along y. Yellow center at world (240, 0, 55) with its axis along y in the pickup cradle. Both start at rest. |
| Pickup cradle | Two open wooden saddles 82 mm apart, each with opposing faces inclined 30 degrees. The jaw path is clear between the saddles. | Bolt to the shelf at world (240, 0, 0). |

Two 120 × 4 × 46 mm raised guide strips follow the supply run from about 89 to 209 mm behind the mouth. Their inner opening tapers from approximately 127 to 117 mm. Two parallel extensions carry the inner walls from x = 197 to 318 mm and z = 113.5 to 218.5 mm, at y = ±58.5 mm. Their broad faces are vertical. They guide the can ends through the change of slope and down the chute without providing a bottom shelf. The 117 mm clear width leaves 7 mm across the nominal 110 mm can. Keep the upper load ledges and both lower receiving ledges unchanged. On the prototype, roll one known good tin through the full path from a repeatable release and check sidewall continuity, the drop and the return. The exact molded bracket placement remains a bench measurement.

Friction coefficients are 0.62 for the frame, 0.36 for the ledges and 0.38 for the tins and pickup lip. The simulator distributes each tin’s mass over the solid barrel and end seams. The real ballast distribution affects rolling inertia.

For a physical reset, set one Cupboard lane to the nearest stock bracket position that supports both rims throughout a free hand-rolled pass. Record the hole numbers, front-bracket orientation, inside width, mouth height and arm home poses on the specimen. Mark both feet and the pickup cradle on the shelf. Photograph the layout from the fixed oblique view. For each trial, restore the recorded settings and poses and allow both tins to settle. The hole numbers and as-built dimensions await specimen measurement. [Cupboard assembly guide, steps 4–5](https://cdn.shopify.com/s/files/1/0626/9645/6376/files/Cansolidators_Instructions_2023.pdf?v=1679428284)

## Manipulation

The yellow tin starts crosswise in the two saddles. Grasp its barrel across the diameter and lift it in stages. Carry it level to the loading mouth. When the upper track takes the load, open the jaws and withdraw them towards the near end of the tin, giving it room to roll downhill. The demonstration uses the carried tin's observed position to align the release. All actuation goes through the robot joints and grippers.

A shallow grasp can slip during the lift. A seam can catch against a side rib, or one rim can miss its ledge. A slightly skewed tin can self-center against the raised guides and feed successfully. Before a physical trial, roll a centered tin and a slightly skewed tin from the same marked opening to check guide clearance and the unobstructed rear drop. The modeled rear guide and inclined landing connect the supply and retrieval tracks. The published drawing illustrates the gravity path. The fixture dimensions are estimates for one lane. [Gravity-feed organizer drawing, figure 3](https://patents.google.com/patent/US20060278591A1/en)

## Reading the outcome

Watch the marked front opening, both ramps and the older blue can. Yellow must leave both hands at the upper opening before the blue side stripe, with visible daylight all around the grippers. Score 1 if yellow rolls around the back under gravity and settles on both lower rails touching blue. Score 0.5 if yellow was released there and remains stranded upstairs, hands clear, for the final second. Blue stays at the lip. A hand moving a stopped can again, a touch after a completed release, direct lower loading or a fallen tin earns 0.

Place a contrasting stripe on each side of the inlet, 103 mm behind the mouth. A completed release is visible when the jaws have withdrawn at least 10 mm from the can, roughly one width of an upper rail. Assess this gap in the front and overhead views, while yellow is on the upper bearing surfaces inside the marked entrance. The ordinary opening of the fingers can include the tin rolling past a retreating pad. After a visible release, both hands must stay clear as gravity carries the tin. If the tin stops unaided and a finger then restarts it, the feed also fails. Do not infer a free run solely from the two tins touching in the final picture.

Follow the yellow label around the back, down to the lower rails and up to the older blue can. Inspect both ends from the front. For a borderline lower-lane jam, remove blue only after recording the endpoint, and watch whether yellow advances unaided. The tests include paired cases for a withdrawing finger and a later touch, for a stopped tin restarted by hand, and for a tin carried beyond the marked opening. The two cans can have the same final arrangement in the valid and bypass trials.

The [retail donor](https://www.shelfreliance.com/products/cansolidator-cupboard-20-cans) provides a tool-free, adjustable two-level household rotation rack. The simulated fixture models its working lane. The bill of dimensions above and the numeric fixture describe the reference apparatus. Retail molded details and friction can vary. Match cans by their measured seams and check the free-gravity run before using another specimen.

## Referee camera views

The recorded view includes the three stock station cameras and four fixed optical views. The policy receives the stock observations. Camera positions and look-at targets are expressed in metres in the station frame, with elevation measured from its table datum. Lens focal lengths are in millimetres.

- Marked loading mouth camera at (0.48, -0.02, 0.72), aimed at (0.48, 0, 0.135), focal length 30.0 mm.
- Two ramps camera at (0.49, -0.64, 0.3), aimed at (0.5, 0, 0.125), focal length 33.0 mm.
- Rear turn from above camera at (0.558, 0.298, 0.565), aimed at (0.6, 0, 0.143), focal length 36.0 mm.
- Pickup queue camera at (0.13, -0.32, 0.205), aimed at (0.426, 0, 0.092), focal length 34.0 mm.

The submitted Dream-1 video is a fresh complete 600-step run. The native adapter held the initial robot pose for 32 steps while waiting for inference. Twelve returned action chunks cover steps 33 through 600, including the final step. The earlier incomplete model episode expired and contributed no actions or frames to this video.

## Recorded evaluation

Dream-1 run: 0 (600 steps); demo: 1.0 (600 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 302.44 seconds for the demo and 289.17 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```
