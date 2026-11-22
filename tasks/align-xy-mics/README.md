# Aim and align an X/Y stereo microphone pair

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 1200 | 360.24 s |
| Scripted demonstration | 1 | 1200 | 357.55 s |
| Independent scripted repeat | 1 | 1200 | 346.26 s |
| Hold initial pose | 0 | 1200 | 333.79 s |

The terminal window spans steps 1161–1200. Angle and separation ranges below are rounded from all 40 samples. Direction checks apply throughout the window.

| Policy | Included angle | Horizontal capsule separation | Upper pickup direction | Lower pickup direction |
|---|---:|---:|---|---|
| Scripted demonstration and repeat | 89.579275–89.579432° | 3.424457–3.424534 mm | Forward and inward | Forward and inward |
| Dream-1 | 59.230083–59.230235° | 39.453830–39.453932 mm | Forward and inward | Forward and inward |
| Hold initial pose | 44.999126–45.002071° | 50.245627–50.247931 mm | Forward throughout, with the inward check failing | Forward and inward |

For every run, each gripper registered 0 N normal contact force on all six assembly components in all 40 samples. The fixed lower collar and captive upper sleeve provide support. The two scripted runs produced identical trace hashes at seed 0.

In the scripted demonstration, the upper capsule’s measured planar radius from the swivel increased from 45.005 mm at step 1 to approximately 67.319 mm throughout the terminal window. The 25.71 mm axial advance in the design record is the nominal target.

Dream-1 held its starting pose for steps 1–32. The action cross-check records 25 model chunk starts, with the first at step 33 and the last at step 1185. Model actions covered steps 33–1200, with zero post-model hold steps.

Independent checks for the blind review were limited to the supplied stills, task card, README and trace summary. The stills show the gross adjustment, with some capsule marks and clip surfaces occluded. The terminal measurements come from the traces.

The design notes below were recorded before these native trials. The table above and validation.json report the measured results.

<details><summary>Original pre-trial design record</summary>

# Aim and align an X/Y stereo microphone pair

The RØDE Stereo Bar guide describes mounting the microphones at different heights, aiming them inward at 90°, sliding them until the capsule centers line up vertically, and securing the adjustments. This station starts with the height offset and lower microphone set. A recording technician uses the geometry to establish stereo pickup before connecting the microphones. The short spacing helps reduce phase cancellation when the two channels are combined into mono. RØDE documents this arrangement in a string-quartet recording. [Stereo Bar quick-start guide, X/Y steps 3–5](https://edge.rode.com/pdf/page/378/modules/1360/sb20_quickstart.pdf) [RØDE quartet recording case](https://rode.com/en-de/about/news-info/recording-a-string-quartet)

## Fixture and reset

Dimensions below are in millimetres. The table surface is z = 0. Positive x is forward and positive y is left. The YAM bases sit at (0, +305, 2.5) and (0, −305, 2.5). Actor frames start parallel to the table frame. The lower microphone’s heading is built into its geometry.

| Component | Construction and nominal mass | Reset placement |
|---|---|---|
| Stereo bar | Bolted aluminum plate, 240 × 240 × 8. It incorporates a Ø20 × 45 left riser and a stepped right riser with a Ø24 × 42 base and a Ø12 × 17.5 spindle support. Total 1.35 kg. | Plate center (350, 0, 4). Riser axes at x = 350 and y = +50 and −50. |
| Lower microphone | Unpowered Ø20 × 100 metal barrel in a locked Ø24 × 12 aluminum collar. Total 92 g. | Collar center (350, +50, 65), with the pickup axis aimed 45° inward and the capsule at (400, 0, 65). The barrel midpoint is 28.71 mm ahead of the collar along that axis. |
| Upper clip and swivel | Anodized aluminum Ø28 × 6 thrust disk joined to a 12 mm long captive sleeve. The sleeve colliders form seven bearing flats and a 1.2 mm crown cap. Total 24 g. | Sleeve center (350, −50, 90). The thrust disk center is 18 mm below the sleeve. The vertical swivel starts at 0° with stops at −10° and +55°. Positive swivel angle is inward. |
| Upper microphone | Unpowered matte blue Ø20 × 100 steel barrel, with an integral 54 × 4 × 1 brake land along the rear part of its top surface. Total 78 g. The land rises 0.7 mm above the nominal cylinder crown. | Midpoint (353, −50, 90), pointing along +x. The capsule starts at (395, −50, 90). Axial travel is −10 to +37 mm from reset, giving capsule radii of 35–82 mm about the swivel center. |
| Slide shoe | Steel-backed flat dry-friction shoe, 6 × 3.4 × 1.5, 1.2 g. | Center (350, −50, 101.85) before settling. A sprung vertical guide in the sleeve seats the shoe on the top brake land. |
| Swivel brake | Keyed brass carrier about 27 mm across and 6 mm high including four bonded 4.6 × 8 × 2 dry-friction pads. Total 9 g. | Carrier datum (350, −50, 64.6) before settling. The pad centers lie at a 10.7 mm radius. A sprung vertical guide around the spindle seats the pads against the bottom face of the thrust disk. |

The RØDE M5 datasheet supports the barrel scale. The fixture specifies a capsule datum 92 mm from each rear rim and 8 mm behind each pickup face. Survey the specimen’s capsule datum before applying the 3 mm gold stripe centered on it. A captive pin and recessed slot establish the slide travel on the bench. The simulation uses a prismatic joint for the guide fit and stops. Its sleeve flats have clearance around the barrel so the intended drag comes from the shoe. [RØDE M5 datasheet](https://edge.rode.com/pdf/products/48/M5_Matched_pair_DS_V02.pdf)

Verify the plate fasteners and the lower clip. Set the upper barrel straight forward with a removable square and set a 41 mm gap between the rear sleeve face and rear rim with a spacer. Remove the square and spacer and let the assembly settle. The task geometry has a fixed start with no task-level random offsets. Use 0 as the nominal replay seed and record the effective runner seed for each trial.

At `yam-arena` revision `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, the reset commands `READY = [0.0, 1.0, 1.0, -0.6, 0.0, 0.0]` radians and normalized jaw opening `1.0` for each arm during settling. `FINGER_OPEN = -0.04695` m corresponds to a commanded 93.9 mm gap. The 50 mm jaw commands in the reference trajectory begin after reset. Report achieved joint angles or gaps only from the native validator’s settled `initialRobotBeforeAction` record in an authorized trial, tied to the exact fixture and adapter run.

## Manipulation and retention

The nominal setting has the upper barrel aimed 45° inward and its capsule 70.71 mm ahead of the swivel. The upper barrel advances 25.71 mm along its own axis. The targets sit inside both travel ranges, and the crossing barrels have 5 mm nominal surface clearance. The reference path grips the exposed barrel 22 mm behind the capsule, takes three short swivel steps and four axial steps, holds the endpoint, and opens in two steps at a stationary target. Once open, the jaws rise and retract together toward the right base in two steps before parking. This puts the lift closer to the arm base while retaining the jaws’ orientation across the barrel. The path uses 1,050 ticks before the final 150-tick park hold. Either arm and either adjustment order are allowed.

The swivel and barrel slide are unpowered joints with zero drive stiffness and damping. The flat shoe and concentric four-pad thrust brake are independent. Their carrier springs load the contact surfaces normal to the desired motion. The 750 N/m shoe spring and 650 N/m thrust brake spring give nominal normal reactions near 4.5 N and 7.7 N from the design dimensions and component weights. These are calculated preloads. Commission resistance with a spring scale pulled along the barrel for axial drag and tangentially at a known lever arm for swivel drag. Proposed bench targets are 2–3 N axial breakaway and 1–2 N running drag, plus 0.03–0.05 N·m swivel breakaway and 0.02–0.035 N·m running torque. The carrier load channel aggregates the four thrust pads. It gives the total reaction for diagnosis.

Check actual jaw, wrist and arm clearance through the descent, grip, turn, slide, opening and retreat. The starting grasp is 23 mm forward of the pivot, so a tool extending 12 mm along the barrel has 5 mm nominal clearance from the sleeve front. Grip slip can leave the upper capsule short, and a jaw can disturb the barrel during release. Record capsule radius, included angle, gripper contacts and the two brake reactions through release to distinguish axial slip from swivel drift. The native scripted trials demonstrated a released hold that met the scoring criteria throughout the final two seconds. Installed clearances, friction and timed retention on a physical bench still require verification.

## Manual scoring

Award 1 if all conditions hold throughout the final two seconds of the 60 s trial. Both pickup axes must point forward and inward with an included angle of 85°–95° inclusive. The horizontal capsule-center separation must be at most 5 mm. Both clips must support their microphones and both grippers must remain clear of the assembly. Otherwise award 0. The different mount heights and captive hardware establish vertical order and support. Simulation uses the fixed collar and captive joints, the body-fixed capsule centers and horizontal barrel headings, and zero measured arm contact on every assembly component in all 40 terminal samples. The brake reactions and velocities are diagnostic measurements. A transient failure before the final window can be recovered by completing a fresh terminal hold.

Read the upper heading with a base-registered protractor and measure the rear-rim gap from the back face of the sleeve with a depth caliper. With inward upper heading θ and gap g in millimetres, the nominal upper capsule radius is r = 86 − g and the included angle is 45° + θ. For L = 70.71 mm, the squared horizontal miss is r² + L² − 2rL cos(θ − 45°). Use the surveyed fixture dimensions and capsule datums for the bench calculation. The nominal aligned gap is 15.29 mm. A 30.29 mm gap at the same heading passes the angle check and leaves a 15 mm capsule miss. Swiveling alone leaves a 25.71 mm miss at the correct angle, and sliding alone leaves at least 50 mm of lateral separation. The adjustment stops lie away from the usable setting.

Record the terminal interval in a registered close view that shows the released jaws, clip seating, heading and rear rim relative to the sleeve. Cross-check the released endpoint using the protractor and caliper without disturbing the assembly. Use surveyed settings just inside and outside the 5 mm limit to commission the instrument readings and verify that the close view can reveal drift during the hold. The 320 × 176 policy view provides the barrel direction, capsule marks and gross fixture geometry for manipulation. Use the close view and instruments for numerical scoring.

## Reproduction

Keep the declarative fixture, scorer cases and English card in the `align-xy-mics` folder. The environment is the fixed dual-YAM tabletop PhysX station under gravity. The robot receives RGB and joint observations. It commands ordinary joint targets and normalized jaw widths. The horizon is 1,200 control steps at 20 Hz. The cases are synthetic measurement streams for the terminal scorer.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

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

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 357.55 s for the scripted run and 360.24 s for the model trial.
