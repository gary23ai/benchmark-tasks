# Pot a pool ball with a bridge hand

Pick up the cue from the rail. Use the left gripper as a bridge on the cloth while the right gripper strokes
the cue, sending the white ball into the yellow 1-ball and potting the 1-ball in the marked corner pocket.
The balls, cue and table are full size. No extra cue support or grasp attachment is used.

## Run and score

Recorded runtime: Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU. Install this folder in that
YAM environment:

```sh
uv pip install --no-deps .
python -m rdb_pool_bridge.run scripted --out out/scripted
python -m rdb_pool_bridge.run dream --out out/dream
```

Configure the Dream Labs client separately for the model run; no credential is included here.
Both commands use the same scorer, fixed layout, seed 0 and zero object jitter. The limit is 1,600 actions
at 20 Hz (80 simulated seconds). The scripted command exits successfully only at full score.

| Recorded run 4 | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted reference | 1.0 | 620 | 159 s |
| Dream-1 | 0.0 | 1,600 | 283 s |

Dream-1 receives only 320 x 176 RGB from the overhead and two wrist cameras, plus joint and gripper positions.
The scripted reference uses simulator object-state feedback and commands only the arms; it does not move or
attach objects by code. It is a physical-feasibility reference, not an observation-matched policy baseline.

Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and
`media/grader-trace.jsonl`. Read `score` in RESULT.json for the final score. The side camera is for review only.
The uploaded videos combine the complete, synchronized overhead and side recordings from the same trial.
Follow the overhead view for the ball path and marked pocket, and the side view for the bridge and stroke.

## Setup, scoring and reset

`task.json` states the fixed object positions, legal-stroke conditions, fouls and numeric partial credit:
0.1 for a legal strike, 0.4 when the white hits the 1-ball first, 0.7 when the 1-ball crosses the target pocket
mouth, and 1.0 when it drops there with the white still on the table. An illegal stroke or foul scores zero.
Both hands must play their specified roles at the strike. There are no alternate submitted layouts.

Reset by re-spotting all three balls and laying the cue on the rail at their marks, then restore the arms to
the ready pose. This takes under a minute and consumes nothing. In simulation a new run resets the scene.

## Reproduction and physics

A real 7-ft home table is required. Hiring a table by the hour is a lower-cost alternative to buying one;
the playing surface and shot remain full size. Put the robot bench against its long rail at the stated
height. Equipment dimensions, material details, sources, geometry and the real-trial scoring procedure are
in [TECHNICAL.md](TECHNICAL.md). Cloth rolling resistance and spin drag use stated per-step forces; contacts,
the stroke and the pockets use PhysX. Balance point, pocket depth and home-table mouth sizes are estimates.

[REFERENCE-SPEC.json](REFERENCE-SPEC.json) preserves the original task card. [validation.json](validation.json)
records the measured scores, matching scene/scorer hashes and archived input identity. The code and assets
match the recorded run bundle. The README runtime version was corrected during packaging; the simulation
and controller were not changed.

## License and checks

Code and procedural assets: MIT (`LICENSE`). No manufacturer CAD, images or textures are bundled.
The YAM station and its robot meshes are external dependencies with their own licenses, listed in TECHNICAL.md.
Run `python -m pytest -q tests` for the local checks. Offline dry runs test controller logic, not contact
physics; the results above come from the recorded Isaac runs.
