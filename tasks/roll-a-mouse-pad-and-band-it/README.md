# Roll a mouse pad up and slip a paper band over it before it springs open

Roll the mouse pad up from one short edge and slide the paper band over the roll, so that the band holds it rolled. Then put the banded roll down, let go and move both arms away.

A loose rubber-backed pad springs open between turns. Start with the pad flat, wind it, and slide a stiff paperboard sleeve over the roll before releasing it. The result can be checked where the sleeve encloses the pad. Manufacturer packaging guidance documents rolled pads retained with bands (Darshion, Why Some Mouse Pads Warp After Shipping).

## Real-trial grading

Scored on the state at the end of the trial (the run limit, or as soon as the 1.0 state is reached); nothing done earlier is held against the robot. On a real trial: note where the grippers are, then photograph the pad and band from above and end-on, and measure with a ruler. The pad counts as a roll if, seen end-on, it winds one way through at least one full turn, so that its two short edges overlap, and nowhere bends back the other way by more than 20 degrees: a pad folded flat in two or zigzag is not a roll, whatever is put round it. 0: the pad is not a roll, or less than a third of the band's length (27 mm) is over it. 0.3: the pad is a roll and the band is started on it: at least a third of the band's length (27 mm) is over the roll. 0.6: the pad is a roll and the band is on it: the whole roll passes through the band and stands at least 10 mm out of each of its ends. 1.0: in addition (a) the roll is square: measured along the roll, the turn that stands furthest out at one end and the turn that stands furthest back from it differ by no more than 10 mm; and (b) the banded roll has been put down and let go: both grippers' centre points have been at least 50 mm away from the band and from the pad for 2 seconds, so that nothing but the band is holding the roll shut. Where it is put down does not matter (the cradle, the table or the carton). A banded roll that is telescoped or still held scores 0.6. Possible scores: 0, 0.3, 0.6, 1.0. Inspect both exposed ends after release. Radial flaring beyond the sleeve is allowed; measure the 10 mm end offset along the roll axis, not across its diameter. The sleeve must surround all layers, and an open flap that unwinds the overlap fails the roll check.

[Manufacturer discussion of bands on rolled pads](https://darshion.com/why-mouse-pads-warp-after-shipping/). For a physical trial, record the two exposed ends against a ruler with hands clear. Read axial offset separately from radial flare. The source documents commercial practice; the 70 mm sleeve and 80 mm width are benchmark dimensions, not manufacturer test results.

## Recorded evidence

The recorded scorer and physical scene are unchanged. At release the recorded pad had 1.086 turns, 4.2 mm axial offset and 22.6 mm minimum protrusion; the whole sleeve surrounded the layers. Dream left the pad flat. These are simulation measurements. The manufacturer describes banded rolls in general, without measuring this fixture.

## Run one trial

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime's installation instructions and source its `yam.env` before running these commands from this folder.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_pad_roll.run scripted --out out/scripted
python -m rdb_pad_roll.run dream --out out/dream
```

Configure Dream Labs credentials privately on the run host before the `dream` trial; none is included. Both commands use Inspect Robots, the same scene and the scorer (`rdb_pad_roll/grading.py`), seed 0 and zero object jitter. Run limit: 2,400 actions (2 minutes at 20 Hz), 240 Hz physics.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 1,429 | 903 s |
| Dream-1 | 0.0 | 2,400 | 1359 s |

One trial of each was run (the simulator was shared with other jobs, so wall times are long). Dream-1 used 99 requests of a cap of 128; at the end of its run the pad still lay flat and the band was in its cradle.

Dream-1 receives only 320 x 176 RGB from the overhead and two wrist cameras, plus the 14 joint and gripper positions. The scripted reference reads simulator object state (the pad's strips and the band) for feedback; it is a feasibility reference, not an observation-matched baseline. It commands the 14 joint and gripper targets only and never writes an object's pose. The review-only side camera is never a model input.

Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read `score` in `RESULT.json`. The scripted command exits successfully only at full score.



## Third-party assets and licences

| Asset | Used for | Source | Licence |
|---|---|---|---|
| YAM arm and parallel gripper: URDF, link meshes, collision hulls | both robot arms | I2RT Robotics [`i2rt`](https://github.com/i2rt-robotics/i2rt), as vendored in `yam-arena` | MIT |
| Station: table, room, camera stand, arm base and camera poses | the bench the arms and cameras stand on | `yam-arena` ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) at the pinned commit) | that repository's README states that it has no licence yet |
| Cameras: one overhead, one on each wrist | the model's three RGB views | simulated pinhole sensors placed by `yam-arena`; no camera model or image is loaded | as the station |
| Isaac Lab-Arena, Isaac Lab, Isaac Sim | simulator and environment layers | NVIDIA / the Isaac Lab Project | Apache-2.0, BSD-3-Clause and the Isaac Sim licence |
| Inspect Robots and its YAM and Dream adapters | trial runner, guards, model client bridge | the pinned packages above | their own licences |

None of the above is bundled in this folder. Everything in it (the carton, the pad, the band and the cradle in `pad.usda`) is generated by this folder's code.

