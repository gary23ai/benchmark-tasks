# Assemble a glass vacuum filter holder with its spring clamp

Assemble the vacuum filter holder. Pick up the glass funnel and set it square on the fritted base, over the white membrane. Keep holding the funnel down with that hand while the other hand squeezes the aluminium spring clamp open, fits it round both glass flanges and lets it close. Then let go of everything and move both arms back.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_filter_clamp.run scripted --out out/scripted
python -m rdb_filter_clamp.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 2,400 steps (120 s at 20 Hz). Use the default layout; additional diagnostic variants are not the submitted trial.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 993 | 250 s |
| Dream-1 | 0.0 | 2,400 | 658 s |

Dream-1 receives only 320 x 176 RGB from the overhead and two wrist cameras, plus joint and gripper positions.
The scripted reference uses simulator object state for feedback; it is not an observation-matched model baseline
or a demonstration of perception from RGB. It commands robot joints and grippers without writing object poses.
The review-only side camera is never a model input.

Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4`
and `media/grader-trace.jsonl`. Read `score` in `RESULT.json` for the final score.
The scripted command exits successfully only at full score. Startup and model latency add wall time.

## Fixed setup, scoring and reset

`task.json` gives every object, the fixed starting layout, numeric partial-credit/failure rules and reset steps.
The setup screenshot uploaded with this task is the recorded starting scene.
`validation.json` records the two scores and matching scene/scorer hashes.

Reset: Take the clamp off (press the handles), lift the funnel off, replace the membrane with a new dry one centred on the frit, stand the funnel on its wipe and lay the clamp shut at its mark; check the flask stands on its mark. In simulation: seed 0, zero object jitter; holder, funnel and clamp return to their start poses and the scorer's memory starts fresh.

## Physics and reproduction

PhysX, 240 Hz physics/20 Hz control. Flask, stopper and base form one free body; the clamp has sprung jaws (0.30 N m/rad, estimated). Convex colliders; funnel bore closed for collision. A stated rule tracks membrane displacement when the seated funnel slides; no membrane deformation. Grasping is by contact, not attachment. Clamp force and friction are estimates, not hardware measurements. Full assumptions: REFERENCE-SPEC.json and TECHNICAL.md.

[TECHNICAL.md](TECHNICAL.md) contains the detailed build list, product/dimension sources, low-cost
reproduction options, physical scoring procedure, controller stages and measured limitations.
[REFERENCE-SPEC.json](REFERENCE-SPEC.json) preserves the full specification used for the recordings.
Only the card title and fidelity summary were shortened; instruction, setup, scoring, reset and run limit are unchanged.

## License and tests

Task code and procedural assets: MIT, see `LICENSE`. No manufacturer CAD, images or meshes are included.
Third-party runtime dependencies retain their own licenses, listed in TECHNICAL.md.
Run `python -m pytest -q tests` for the local checks. Dry runs are unit-level checks, not simulator evidence;
the scores above come from the recorded Isaac runs.
