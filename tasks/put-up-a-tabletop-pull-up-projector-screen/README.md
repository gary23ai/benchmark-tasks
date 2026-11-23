# Put up a pull-up projector screen

A portable pull-up projector screen lies closed on the table. The robot has to put it up: pull the screen up by its
top bar's handle (its roller spring pulls it back in), keep it up while pulling the telescoping support post up to
its full height, where it clicks, then let the screen down so that the tab on its top bar drops over the pin on the
post, and take both hands away.

Instruction given to the model:

> Put the projector screen up: pull it up and hang it on its support post at full height. Then take both hands away.

`task.json` has the full setup, scoring, reset and fidelity text.

## Run and score a trial

The task is one `yam_arena` task file, `put_up_a_tabletop_pull_up_projector_screen.py` (scene, mechanism, scorer and
scripted demo), with `bodykit.py` next to it. It runs in the YAM Isaac Lab-Arena runtime
([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1):
Isaac Sim 6.1, Isaac Lab-Arena 0.3.1) with `inspect-robots==1.0.0` and `inspect-robots-yam==0.38.0`.

Scripted demo (reaches full score):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=put_up_a_tabletop_pull_up_projector_screen.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the projector screen up: pull it up and hang it on its support post at full height. Then take both hands away." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir runs/screen-demo
```

Dream-1 (needs a Dream Labs key in the environment, `DREAM_API_KEY`, or in `~/.dreamlabs/credentials`): the same
command with `--policy dream` and `--log-dir runs/screen-dream`.

Videos, one MP4 per camera: `inspect-robots video runs/screen-demo/<log>.json`.

The episode can finish when both hands have been clear of the hung screen for a continuous two seconds. The timer uses the actual simulator control interval. Set `SCREEN_DEBUG=1` to print bar/post height, latch state, hand clearance and score. To reproduce the post-only checks, prefix the scripted command with `SCREEN_POST_ONLY=1`, and add `SCREEN_POST_SLOW=1` for the longer lift. For a bar-only control, set `SCREEN_ONE_HAND=1`. Use a separate output folder per trial and keep object jitter at zero.

## Recorded trials

| Native trial | Score | Control steps | Sampled peak bar rise | Sampled peak support rise |
|---|---:|---:|---:|---:|
| Support knob only, ordinary lift | 0.0 | 520 | 248.2 mm | 263.2 mm |
| Support knob only, five-second lift plus three-second hold | 0.0 | 620 | 246.7 mm | 261.7 mm |
| Full scripted setup | 1.0 | 381 | 342.7 mm | 320 mm |

The two post-only controls kept the other arm at its starting pose. Half-second traces for both controls and the full demonstration are in `evidence/`. The peak heights are sampled every half second. The bar can ride on the shoulder while the post is pulled. The rubric evaluates a usable, independently supported screen and allows either hand to operate it. A closed-case Dream-1 trial ran for 1,000 actions, never reaching either partial-credit condition; the original video is included. An archived bar-only trial raised the screen but did not lock the post (0.25). The new full-score video uses every overhead frame of the submitted reference run. Seeds, timing and artifact hashes are in `validation.json`. Typical GPU wall time varies with load.

## Scoring a real trial

Mark the starting heights of the top bar and support shoulder with a ruler. Use side and overhead video and an end photo. Award the highest verified tier. 0.25 if at any point the bar rises at least 295 mm or the support locks at full extension (320 mm travel, 3 mm detent tolerance). 0.5 if both conditions hold together at any point. For 1.0 the bar must be 305 +/-10 mm above rest and its tab must be seated around the pin, with at most 4 mm between the underside and the locked shoulder. Both gripper centres must stay at least 50 mm from the complete bar centreline, exposed post axis and knob centreline while the screen remains hung for 2 continuous seconds. A break restarts the timer. All other outcomes score 0 unless a partial tier was reached.

Film the support and bar in profile against ruler marks and use an overhead view for clearance. The shoulder can visibly carry the tab during extension. A reference from a full-size portable screen shows the lift-and-hang operation and rod bracket. The tabletop dimensions and forces here are those of the specified reconstruction. [Elite Screens ezCinema user guide](https://elitescreens.com/wp-content/uploads/UG_ezCinema.pdf)

## Reset

Lift the screen off the post and let it roll back into the case, press the post's release and push it back down, and
set the case back on its tape marks. In simulation each trial starts the simulator afresh.

## Measuring the real screen

The roller spring's pull (about 5 to 7 N here), the post's return spring and its click height are estimates: measure
them on the bought screen with a spring scale and a ruler.

## Third-party assets and licences

None of the following is bundled in this folder; each comes with the pinned runtime above.

| Asset | Used for | Source | Licence |
|---|---|---|---|
| YAM arm and parallel gripper: URDF, meshes, convex collision hulls | both robot arms | I2RT Robotics [`i2rt`](https://github.com/i2rt-robotics/i2rt), commit `120c3c8`, as vendored in the runtime under `yam_arena/assets/yam/` | MIT (`LICENSE.i2rt` there) |
| Station: table, room, camera stand, arm base and camera poses | the bench the arms and cameras stand on | the pinned yam-isaac-lab-arena runtime (`yam-arena`), `station` scene, boxes and cylinders; no mesh | that repository's README states that it has no licence yet |
| Cameras: one overhead and one on each wrist | the model's three RGB views | simulated pinhole sensors placed by `yam-arena`; no camera model or image is loaded | as the station |
| Isaac Lab-Arena, Isaac Lab, Isaac Sim | simulator and environment layers | NVIDIA / the Isaac Lab Project | Apache-2.0, BSD-3-Clause and the NVIDIA Isaac Sim licence |
| Inspect Robots and its YAM and Dream adapters | trial runner, scripted and Dream-1 policies | `inspect-robots`, `inspect-robots-yam`, `inspect-robots-dream` at the pinned versions | MIT |

**Reused task code:** `bodykit.py` is copied unchanged from the accepted task "Reload a long-arm stapler and staple
two sheets together" in robocurve/benchmark-tasks (`tasks/reload-a-long-arm-stapler-then-staple-two-sheets/`). It
builds rigid bodies from several boxes and cylinders and the hinges and slides between them. The helper functions for
the scripted demo's inverse kinematics and joint-space lead-ins follow the same task's. Credit to its authors.

Everything else in this folder (the screen case, screen, post, the scorer and the demo) is this folder's code and is
released under the MIT licence. No manufacturer's model, drawing, photograph or texture is used.
