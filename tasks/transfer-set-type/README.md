# Transfer a line of loose wood type from a composing stick to a galley

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 600 | 229.24 s |
| Scripted demonstration | 1 | 474 | 223.78 s |
| Independent scripted repeat | 1 | 474 | 227.01 s |
| Hold initial pose | 0 | 600 | 210.71 s |

The design notes below were recorded before these native trials. The table above and validation.json report the measured results.

<details><summary>Original pre-trial design record</summary>

# Transfer a line of loose wood type

A compositor needs the stick free for the next line and the letters still in their setting. Leeds Print Workshop describes releasing the stick and pinching composed matter during transfer to a galley. The Cowper press manual describes lifting and carrying set type under end pressure. The four-letter specimen, gap, time limit and scoring tolerances are engineering choices for this task. [Leeds Print Workshop, section 2](https://leedsprintworkshop.org/wp-content/uploads/2022/04/Letterpress-at-Leeds-Print-Workshop-notes.pdf) · [Cowper press instruction manual](https://upload.wikimedia.org/wikipedia/commons/d/d7/Printing_apparatus_for_the_use_of_amateurs_-_containing_full_and_practical_instructions_for_the_use_of_Cowper%27s_parlour_printing_press_%28IA_printingapparatu00holtrich%29.pdf)

## Build and reset

All dimensions and coordinates below are in millimetres. In the tabletop frame, x points forward, y left and z up. The left and right YAM bases are at (0, 305, 2.5) and (0, −305, 2.5). All objects start aligned with the tabletop axes. Each fixture datum is on its upper floor surface. Each letter datum is at the center of its foot.

| Object | Material, size and mass | Initial placement |
|---|---|---|
| Composing stick | Bench-mounted aluminum, 180 g. The 38 × 66 × 4 plate stands on two 30 × 8 × 36 feet. The back rises 12 mm. The fixed end and released knee each rise 6 mm. | Floor center (306, 0, 40). Back contact plane x = 292. Open lip x = 325. End contact planes y = +24 and −28. |
| Galley plate | Bench-mounted aluminum, 250 g. The 70 × 100 × 4 plate stands on two 60 × 12 × 36 feet. | Floor center (405, 0, 40). Open lip x = 370. |
| Galley rim | Aluminum, 16 g. Two 68 × 2 × 12 side rails meet a 2 × 100 × 12 back rail. The rim is screwed to the stationary plate. | Same datum as the plate. Inner side faces y = ±48. Inner back face x = 438. |
| A, B, C and D | Four loose end-grain maple sorts, 3.2 g each. Each shank is 17 mm deep, 12 mm wide and 22.3 mm high. The raised glyph adds 1 mm. A 9 × 14 mm painted patch identifies the front. | Foot centers (300.5, y, 40), with y = +18, +6, −6 and −18 in letter order. Patches face +x. |

Empty the galley. Seat the letters flat against the stick back in red, blue, green, ochre order from the fixed end. Adjacent shanks touch. Use a millimetre rule to check the 48 mm line, the 4 mm clearance at the released knee and the 45 mm gap between lips. Lay a straightedge across the lips to check the common floor height. Friction coefficients are 0.58 for maple and 0.45 for the fixtures.

The simulator supplies the arm reset configuration. The first scripted segment commands both jaws open to 72 mm with fingers pointing down and closing axes parallel to y. The left and right fingertip targets are (300.5, 0, 100) and (405, −30, 100), respectively, 60 mm above the floors.

## Candidate transfer

The script parks the right arm and lowers the open left gripper over the two exposed ends. Its pickup target is 14.5 mm above the stick floor, leaving 7.8 mm of shank above the tip and 2.5 mm nominal clearance above the back. It closes toward 41 mm across the 48 mm line, lifts in stages, carries the line to the middle of the galley, lowers it, opens and withdraws. The waypoints occupy 528 ticks, leaving 72 ticks at the final target. Either arm may assist.

The middle two sorts weigh about 0.063 N together. With symmetric loading and negligible acceleration, each outer-to-inner interface carries about 0.031 N vertically. At the specified wood friction coefficient this requires about 0.054 N of compression per interface. A jaw that catches on a fitting can leave the ends in the grasp while a middle letter slides down. An early release can disturb the setting after an intact crossing.

## Scoring and inspection

**Preservation during transfer.** Track the original A–B, B–C and C–D pairs from reset. Their opposing side-face centers coincide at reset, 11.15 mm above the feet. A sampled 3D separation of at least 4 mm at any pair, or an angle of at least 15° between its front directions or its up directions, is an irreversible break. A common tilt or turn preserves these relative measurements. For example, if C moves 2.4 mm forward and 3.2 mm down relative to its neighbors, its separation reaches exactly 4 mm. Restoring C before release gives the same final appearance as an intact transfer and still fails the task.

**Received setting.** In A’s frame the expected B, C and D foot centers are (0, −12, 0), (0, −24, 0) and (0, −36, 0). Each must finish within 2 mm of its expected point, inclusive. Align a rule with A’s width and a straightedge with its front to inspect spacing and setback. Three gaps of 0.7 mm displace D by 2.1 mm. A stagger of C alone can spoil the setting while the outside span stays at 48 mm.

**Support and release.** Each letter must stand within 8° of upright with its foot center within 1.5 mm of floor height. All four foot corners of each letter must fall within the closed x interval from 370 to 440 mm. The solid side rails bound the floor in y. Each letter has a separate reading for contact with the galley plate. Fence contact contributes nothing to that reading. Each letter requires at least 0.015 N of plate reaction and at most 0.002 N of contact from each arm. Its speed must be at most 8 mm/s and its angular speed at most 8°/s. All receipt conditions must hold simultaneously for 20 consecutive ticks.

Follow the colored shanks in the stock views to inspect order and relative motion. Check the foot line at release and look across the open lip for overhang. A jaw can obscure an interior letter at 320 × 176, and a slip between 20 Hz samples can escape detection. For the evidence gate, compare repeated intact transfers with a matched middle-letter slip followed by repair. Align the neighboring face distances with the video at the slip and inspect all four plate reactions together at release. Loaded clearance, readable boundaries and physical completion within the limit await those trials.

## Reproduction

Use the single task folder `transfer-set-type`, seed 0 and the fixed dual-YAM PhysX tabletop environment with gravity and rigid frictional contact. The robot observes RGB and robot joint state and commands joint targets and normalized gripper width. The limit is 600 ticks at 20 Hz, or 30 seconds. From the task folder, use the following commands.

```
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

The supplied cases are synthetic measurement sequences for the scorer. Independent run evidence, including authentic recordings, scores and a screenshot, belongs to the subsequent evidence gate.

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

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 223.78 s for the scripted run and 229.24 s for the model trial.
