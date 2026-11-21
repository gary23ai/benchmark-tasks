# Seat a saucepan handle in its holder for one-handed stirring

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 600 | 229.67 s |
| Scripted demonstration | 1 | 494 | 251.7 s |
| Independent scripted repeat | 1 | 494 | 229.35 s |
| Hold initial pose | 0 | 600 | 206.53 s |

The design notes below were recorded before these native trials. The table above and validation.json report the measured results.

<details><summary>Original pre-trial design record</summary>

# Seat a saucepan handle in its holder

An anchored handle holder lets a person stir with one hand while the work surface carries the saucepan. North Coast Medical instructs the user to attach the NC28221 by its suction feet and place the pan handle in the steel-wire slot. The task uses an empty pan on a cold, unpowered surface and ends at release. [Manufacturer’s description](https://www.ncmedical.com/products/pan-holder_263.html)

## Layout and reset

All dimensions below are in millimetres. X points away from the operator, Y to the operator’s left and Z up from the simulator tabletop. All body frames start unrotated. Use seed 0 and the standard dual-YAM home configurations. The left and right bases are at (0, 305, 2.5) and (0, −305, 2.5).

| Object | Construction and placement |
|---|---|
| Worktop surround | Fixed 480 × 580 × 12 laminate-faced board, 1.60 kg, frame at (400, 0, 6). The front edge is at X = 160. The 212 × 212 opening is centered at (360, 0). |
| Weighing plate | Bench-mounted 210 × 210 × 6 laminate-faced aluminum plate on a 180 × 180 × 6 base, 0.94 kg together, frame at (360, 0, 6). The face is flush with the surround at Z = 12, with a 1 mm seam on each side. A 182 mm painted disk marks the spot 200 mm behind the front edge. |
| Saucepan | Free 0.56 kg assembly. The aluminum base is 160 mm across and 4 mm thick. Sixteen 2.8 × 31.5 × 68 tangent panels form the open wall. A 32 × 20 × 14 shank joins a 122 × 24 × 20 rigid phenolic grip. Place the underside center at (360, 0, 12) with the handle along −X. The handle centerline is at Z = 75. Its tip projects 140 mm beyond the base and overhangs the front edge by 20 mm. |
| Suction mount | Two secured rubber feet with bosses, 0.032 kg together, frame at (360, −170, 12). Each foot is 44 mm in diameter and 7 mm high. Each boss is 15 mm in diameter and 14 mm high. The foot bottom centers are (335, −170, 12) and (385, −170, 12), both on the surround. |
| Wire holder | Fixed steel frame, 0.071 kg, frame at (360, −170, 12). The 8 mm round bridge spans 58 mm tip to tip along X with its axis 25 mm above the working face. The two 70 mm rounded uprights stand over the foot centers, 50 mm apart. Their tops are 91 mm above the working face. |

Tare the empty plate. A test weight should register on the plate and leave the plate at tare when placed on the holder mount. Restore the pan at rest in the specified pose. Check that the feet are secure and return both arms home.

## Scripted path

The left jaws approach the rim at (414.8, 54.8) and the right jaws approach the handle at (228, 0). Both approach from above and close with their fingertip targets at Z = 58. The first two loaded moves each command 42.5 mm toward +Y. The intended slide leaves the pan center at (360, 85). Six 15° increments then turn the handle toward −Y. At that pose its tip is 35 mm short of the post plane. Straight advances of 25, 25, 20 and 15 mm feed the grip endwise into the slot and return the base to the mark. The jaws open and withdraw. The trajectory allocates 531 ticks to these motions and 69 ticks to the final hold.

In the nominal seat, the handle extends 50 mm beyond the post plane. The 24 mm grip has 9 mm of clearance on either side within the 42 mm opening. The grip underside is 53 mm above the working face and the bridge crown is 29 mm above it, giving **24 mm of vertical clearance**.

A finger can strike the rim during approach, a grasp can slip as translation begins, or the base can catch at a raised seam. A small yaw error can then send the tip outside a post. For path diagnosis, read the four individual pad reactions alongside `center_y` and the pan pose. Closure ends at tick 114. The intended signed center displacements at ticks 148 and 182 are +42.5 and +85 mm. Each pad reading covers its whole rigid link. Any grasp or entry path that produces the released outcome qualifies.

## Manual scoring

Award one point if all conditions below hold together for 20 consecutive samples at 20 Hz within the 600-tick limit. Otherwise award zero. Bounds include their endpoints.

- **Base support.** The underside center is within 15 mm horizontally of the disk center and within 2 mm vertically of the plate face. Rim tilt is at most 3°. Direct pan-to-plate reaction is at least 4.95 N, approximately 505 g on the tared plate.
- **Release and settling.** Each arm’s total normal contact with the pan is at most 0.10 N. Pan speed is at most 5 mm/s and angular speed at most 2°/s. A sample outside any bound interrupts the dwell.
- **Handle capture.** Read the grip centerline at 140 and 202 mm from the base center along the handle. Both stations must lie between the post axes in X and 30–82 mm above the working face. In Y, the 140 mm station must be at least 4 mm on the pan side of the post plane and the 202 mm station at least 4 mm beyond it. This brackets a continuous section of rigid grip at the height of the wires and allows normal side play.

Use a calibrated overhead view for base center, post axes and the horizontal crossing. A low view along the handle with a millimetre scale reveals a perched grip and gives rim slope and height. Read the tared plate with other bodies clear of its face. Check release and motion over the same second, using contact readings for ambiguous light touches.

## Fixture qualification and limits

The closest false success is a level, fully supported pan on the same mark with the grip tangent to the outside of a post. It can turn away from that post. Compare this placement with an inside seat using matched gentle tangential pulls at the same handle station, from separate resets in each direction. Choose a spring-scale load that turns the free pan while the suction feet stay fixed. In the seated placement each post should arrest its corresponding turn after the available play. Check a perched grip as a third placement. Verify that all three released states can be distinguished in the actual 320 × 176 views. The pulls belong to fixture qualification after the operator trial.

The fixed suction attachment, wire bends, masses, friction and fitted pan dimensions are model assumptions. The rigid pan assembly shares one contact material. Stock fit, visibility at recording resolution and successful manipulation within the time limit await independent trials. The cases specify analytical scorer expectations. Independent run evidence and media belong to the subsequent evidence gate.

## Environment and commands

Use the fixed dual-YAM/PhysX runner with gravity, RGB and robot joint observations. Actions are ordinary joint targets and normalized gripper width. From the `seat-pan-handle` task folder, use the following commands.

```text
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

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 251.7 s for the scripted run and 229.67 s for the model trial.
