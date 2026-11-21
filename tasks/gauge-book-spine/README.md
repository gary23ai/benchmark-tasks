# Set a bookbinder’s spine gauge on a rounded book block

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 600 | 244.69 s |
| Scripted demonstration | 1 | 600 | 266.72 s |
| Independent scripted repeat | 1 | 600 | 257.51 s |
| Hold initial pose | 0 | 600 | 234.34 s |

In the scripted demonstration, both brass tips bore on opposite flats throughout steps 581–600, the final second at 20 Hz. The table gives the minimum normal force and the number of valid button-center projections over those 20 samples. Positions are relative to the block frame, with x measured from the start of the flats and z from mid-height. Values are rounded.

| Tip and face | Minimum force (N) | x range (mm) | z range (mm) | Valid projections |
|---|---:|---:|---:|---:|
| A, positive-y flat | 1.421 | 13.745–13.764 | 3.930–4.362 | 20/20 |
| B, negative-y flat | 1.506 | 13.683–13.709 | 4.262–4.726 | 20/20 |

Both projections stayed within the flat rectangle (x from 0 to 85 mm and z from −75 to +75 mm). Both centers stayed on the exterior side of their respective planes throughout, with A at y ≈ +23.00 mm and B between −22.93 and −22.90 mm. Both minimum forces exceeded the 0.05 N bearing threshold.

Subtracting the modeled 7 mm steel-button diameter from the recorded center distance gives a minimum clear opening of **52.610 mm** and a maximum of **52.699 mm**. The opening stayed within the inclusive 51–55 mm band across all 20 samples. The 53 mm reference comes from the simulated gauge geometry. The trace recorded the dwell milestone at step 600.

The design notes below predate the native trials.

<details><summary>Original pre-trial design record</summary>

# Set a bookbinder’s spine gauge on a rounded book block

The binder fits the brass buttons to the full thickness of the block and can transfer the opposite opening to the flexible spine inlay while the gauge is still in place. The rounded shoulders give the measuring end a plausible place to stop early. [Schmedt operating instructions](https://www.schmedt.de/ww2000/pdf/d-anleitung_falzlehre.pdf)

## Bench layout

Use one `gauge-book-spine` folder and seed 0. The following dimensions and coordinates are in millimetres. The tabletop is z = 0, x points forward and y points left. The left and right YAM bases are at (0, +305, 2.5) and (0, −305, 2.5). All actor frames start with identity orientation.

| Object | Size, material, mass and initial placement |
| --- | --- |
| Foot clamp | Bench-mounted aluminum, 0.240 kg. Its 126 × 80 × 6 base is centered at (456, 0, 3). Two 72 × 6 × 23 cheeks meet the sides at y = ±20. A 6 × 40 × 23 stop meets the fore-edge at x = 503. |
| Practice block | Sealed hardwood, 95 × 40 × 150 overall. The two bonded 0.181 kg halves share the frame (418, 0, 81). The tail rests at z = 6 and the head is at z = 156. The exterior flats extend from x = 418 to 503 at y = ±20. In end view the crown reaches x = 408, with 4 mm nose radii, connecting facets and 6 mm shoulder radii. A 4 mm painted band begins at each transition. The foot clamp secures the assembled block. |
| Gauge rests | Two bench-fixed hardwood bars, each 14 × 100 × 75 and 0.078 kg. Their centers are (261, 0, 37.5) and (340, 0, 37.5). |
| Gauge leaves | Satin steel, 0.098 kg lower and 0.100 kg upper. Each leaf extends about 147 mm along x at reset, with a 55.9 × 9 × 12 rear shank, 15 mm pivot boss, 32 × 8 × 12 forward tine and integral 7 mm spherical output button. Both frames begin at the pivot (295, 0, 81). The output centers are (215, ±40, 81). The vertical hinge starts at 0° and allows −15° to +35°. Spring stiffness is zero. Nominal damping is 0.00009 N·m·s per degree. |
| Measuring buttons | Two bonded brass spheres, each 6 mm in diameter and 0.00096 kg. Their centers start at (355, ±30.5, 81). The positive-y brass button belongs to the leaf with the negative-y rear shank. |

For reset, seat the block on its tail with the fore-edge against the stop. Lay the rear shanks across the rear rest and the tines across the front rest, with their undersides at z = 75. Center the pivot on y = 0. Check 55 mm between the inner brass surfaces and 50 mm from the forward brass surfaces at x = 358 to the crown at x = 408. Return both arms to the simulator’s standard home pose.

## Scripted fit

The script approaches the marked shanks at (229, ±33, 81) with a 22 mm gripper opening, closes across their 12 mm height and lifts 8 mm. During the second lift both wrists rotate 8° about the negative world y axis. This pitches the measuring end upward under load. The open gauge then advances 71 mm, placing the nominal brass centers about 8 mm onto the flats before closure.

Three progressively closer targets turn the grippers with the shanks. The last move supplies closing travel beyond the unloaded square-fit angle, allowing the specimen to take up the load. The final fingertip targets are (300.5, +8.4, 112.3) and (300.5, −7.6, 112.3). They take effect by sample 525 and are repeated for the remaining 75 samples. Continued hand support lets the binder inspect both seats and read the steel opening.

For a native check, compare synchronized samples at grasp closure (195), the first lift (230), the second lift (280), advance (395), and the closing targets (435, 480 and 525). The left shank carries button B and the right shank carries button A. Add 81 mm to each button’s relative height before comparing it with its shank. Read both finger reactions on the same leaf at that sample. Increasing shank height with a low button indicates pitch or slip. A lower finger catching a rest can also disturb the carry. During final closure, check the clearance between the opposing fingers and inspect both brass-to-flat reactions through sample 600.

## Reference and scoring

Seat the physical gauge squarely on a measured 40 mm parallel block. Take repeated inside-caliper readings between the steel buttons and record W, the reading spread and the instrument resolution. For the modeled gauge, the fitted brass spheres have 46 mm between centers. If each leaf turns through θ from reset, their separation is 2(30.5 cos θ − 60 sin θ) mm. At θ ≈ 6.96°, the steel centers are 2(40 cos θ − 80 sin θ) ≈ 60.01 mm apart. Subtracting the steel diameter gives a nominal clear opening of 53 mm.

Award full credit when one brass button bears on each exterior flat and the clear steel opening stays within W ± 2 mm for the entire final second. Otherwise the grade is zero. Inspect both seats along the head-to-tail axis. The crownward edges of the bands identify the first eligible points on the flats. A thin feeler can resolve a doubtful gap. Check the span with inside calipers without shifting the tool. A readable scale in the gauge plane permits a span check across the recorded hold.

At 20 Hz the final second comprises samples 581–600 inclusive. The simulator requires 0.05 N on each of two distinct button-to-block contact pairs. Each spherical center must project onto its flat rectangle and lie on its exterior side. Either assignment qualifies. The accepted steel-center distance is 58–62 mm, equivalent to 51–55 mm between surfaces. The hand and shank channels are diagnostic. The synthetic cases exercise reset, free-space presetting, shallow seating, shoulder contact, both jaw assignments, the span boundary and interrupted bearing.

For a physical counterexample, preset the steel opening to W with the brass end just in front of the crown. The reading can be exact while both faces show daylight. Then advance that same setting onto the flats. For a second control, close on the rounded shoulders and measure the steel opening. Very near the transition it can already fall inside the tolerance, making the contact location decisive.

## Environment and evidence

The fixed dual-YAM PhysX environment supplies gravity, friction and rigid contact. The policy observes RGB and robot joints and commands joint targets and normalized gripper widths. The trial limit is 600 control steps at 20 Hz. From the task folder use

```text
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

The tool outline, button dimensions and pivot resistance are engineering approximations. Commercial-gauge measurements, physical W and repeatability at the scoring boundaries remain open.

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

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 266.72 s for the scripted run and 244.69 s for the model trial.
