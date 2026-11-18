# Homogenize a sample with 15 strokes of a glass Dounce grinder

Homogenize the cells in the glass Dounce grinder with the tight pestle standing in it. Push the pestle down to the bottom, then give it 15 full strokes: each time draw it up until the blue ring on its shaft is above the tube's rim, keeping the red ring below the rim so that the head stays in the liquid, and push it back down to the bottom. Hold the tube down in its cold block all the while, or the pestle will pull it out. After the 15th stroke leave the pestle on the bottom and let go of both.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_dounce.run scripted --out out/scripted
python -m rdb_dounce.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 3,000 steps (150 s at 20 Hz). Use the default layout; additional diagnostic variants are not the submitted trial.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 1,451 | 343 s |
| Dream-1 | 0.0 | 3,000 | 747 s |

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

Reset: The sample is consumed: discard it, rinse tube and pestle with water and then cold PBS, put a fresh suspension in the tube, return the block to the fridge between trials, stand the pestle in the tube with its head at 30, and return both arms to ready with grippers open. In simulation: seed 0, zero object jitter; the tube and the pestle return to their start poses and the scorer's memory starts fresh.

## Physics and reproduction

PhysX, 240 Hz physics/20 Hz control. Rigid glass; head collider has 0.65 mm radial play (real 0.025 mm). A free-axis D6 joint models equal/opposite axial liquid drag, capped at 8.6 N. Authored damping is 250 N s/m; measured effective damping is about 54 N s/m. Foam is a speed rule, not fluid simulation. Grips, strokes and hold-down use contact only. Full assumptions: REFERENCE-SPEC.json and TECHNICAL.md.

[TECHNICAL.md](TECHNICAL.md) contains the detailed build list, product/dimension sources, low-cost
reproduction options, physical scoring procedure, controller stages and measured limitations.
[REFERENCE-SPEC.json](REFERENCE-SPEC.json) preserves the full specification used for the recordings.
Only the card title, context and fidelity summary were shortened; instruction, setup, scoring, reset and run limit are unchanged.

## License and tests

Task code and procedural assets: MIT, see `LICENSE`. No manufacturer CAD, images or meshes are included.
Third-party runtime dependencies retain their own licenses, listed in TECHNICAL.md.
Run `python -m pytest -q tests` for the local checks. Dry runs are unit-level checks, not simulator evidence;
the scores above come from the recorded Isaac runs.
