# Square a skewed stack of loose sheets on both axes and secure it with a binder clip

Square up the stack of paper so that all its edges are flush, and fasten it with the binder clip on the middle of one short edge. Then let go and move both arms away.

The pile starts uneven along both pairs of edges. Bring the pages flush, keep them together while opening the clip, and leave the whole stack secure enough to lift by the clip. A clip can grip only the top pages even when it looks correctly placed.

## Real-trial grading

Judged at the end of the trial (the run limit, or as soon as the robot has let go with the 1.0 conditions met) from a photo, with a steel rule and one lift test. Let go: for the last 2 s no part of the robot has touched the paper or the clip and both gripper centre points have been at least 50 mm from them. Clamped: a sheet lies between the clip's shut jaws, the middle of their lips at least 3 mm in from the sheet's edge. Lift test (by the referee after the end, not by the robot; only if let go): the clip is taken by its back and lifted until the whole stack hangs from it, its lowest corner 152 mm above the table (50 mm above the reams), and held 2 s. Held sheets: after a lift test, the sheets still clamped that slid no more than 3 mm under the lips (a sheet that slid shows as a step at the stack's edges); without one (not let go), the sheets clamped. Flush, read before the lift test: on the clipped stack, measure on each of its four sides the step between the sheet edge that sticks out most and the one that sits back most; take the larger of the two long sides and the larger of the two short sides. Clip placed: the clip is on a short edge (its jaws point along the long side within 25 degrees) and the middle of its lips is within 20 mm of the middle of that edge. 0 for the whole trial if a sheet ever leaves the table (a corner beyond the table's edge or more than 30 mm below its top). Count only full-size sheets lying unfolded in the clipped stack. Temporary fanning while jogging the stack is allowed. Otherwise 1.0: let go, all 36 sheets held, both flush values at most 2 mm, clip placed. 0.6: let go, all 36 sheets held, both flush values at most 6 mm. 0.3: at least 18 sheets held. 0: anything else, including nothing done. Possible scores: 0, 0.3, 0.6, 1.0. In simulation the 36 sheets are nine rigid plates of four, each counting as four sheets. Which hand does what, how the stack is squared and where it is left are not scored.

## Recorded evidence

The recorded zeroing flag stayed empty in both videos and all three controls. Removing the temporary splay penalty therefore changes none of their scores. The lift, sheet-count and flush checks use the original recorded measurements.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime's installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_paper_jog.run scripted --out runs/scripted
python -m rdb_paper_jog.run dream --out runs/dream
PAPER_JOG_DEMO=no_jog python -m rdb_paper_jog.run scripted --out runs/no-jog     # a control
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use
Inspect Robots and the scorer in `rdb_paper_jog/grading.py`. The scripted controller may read simulator
state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and
`media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace gives the scorer's measurements at
every step. A policy receives the overhead and two wrist RGB images at 320 × 176 plus the standard joint and
gripper position readings. The side camera is recorded for review only. Grader state and contact forces are not
model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 2,400 steps (120 simulated
seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 1,435 / 2,400 | 953 s |
| Control `no_jog` | 0.3 | 2,400 / 2,400 | 1557 s |
| Control `one_direction` | 0.3 | 2,400 / 2,400 | 1201 s |
| Control `release_before_clip` | 0.0 | 2,400 / 2,400 | 1112 s |
| Dream-1 | 0.0 | 2,400 / 2,400 | 1069 s |

One run each on the recorded scene, one simulation at a time on a GPU shared with other jobs. Dream-1: both grippers went to the stack and hovered at its near edge for most of the run, the left one touched the paper once (step 1,685) and shifted it by under 2 mm, nothing was grasped and the clip was never approached. Each control is the demonstration with one thing left out (see results/qualification-summary.json); none may reach 1.0. Same-seed repeats of the demonstration have not been run on the final files.

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings,
scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```



## Third-party assets and licences

None of the following is bundled in this folder; each comes with the pinned runtime named under "Run and inspect
the result".

| Asset | Used for | Source | Licence |
|---|---|---|---|
| YAM arm and parallel gripper: URDF, link meshes, convex collision hulls, and the split link meshes that colour the arms | both robot arms | I2RT Robotics [`i2rt`](https://github.com/i2rt-robotics/i2rt), commit `120c3c8`, as vendored in the runtime under `yam_arena/assets/yam/` | MIT (`LICENSE.i2rt` in that directory) |
| Station: table, room, camera stand, arm base poses and camera poses | the bench the arms and cameras stand on | the pinned [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) runtime (`yam-arena`), `station` scene, built from boxes and cylinders; no mesh | that repository's README states that it has no licence yet |
| Cameras: one overhead and one on each wrist (the model's three RGB views), and the side review camera | observations and the review video | simulated pinhole sensors placed by `yam-arena` where the station's overhead and wrist cameras sit; the side camera is placed by `rdb_paper_jog/fixed_scene.py`; no camera model, mesh or image is loaded | as the station; the side camera is this folder's code (MIT) |
| Isaac Lab-Arena, Isaac Lab, Isaac Sim | simulator and environment layers | NVIDIA / the Isaac Lab Project | Apache-2.0, BSD-3-Clause and the NVIDIA Isaac Sim licence |
| Inspect Robots and its YAM and Dream adapters | trial runner, guards, model client bridge | `inspect-robots`, `inspect-robots-yam`, `inspect-robots-dream` at the pinned versions | MIT |

Everything in this folder (the 36 sheets, the binder clip, the two reams, the fin and its foot in `paper_jog.usda`) is
generated by this folder's code and is under the MIT licence in `LICENSE`. No manufacturer's model, drawing,
photograph or third-party texture is used.


