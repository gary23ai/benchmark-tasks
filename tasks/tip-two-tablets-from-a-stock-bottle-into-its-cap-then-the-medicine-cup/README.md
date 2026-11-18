# Pour two tablets into a held bottle cap, then into a medicine cup

Give two tablets from the bottle without touching them: hold the bottle's cap in one hand and, with the other, tip two tablets from the bottle into it; tip any extra from the cap back into the bottle, then tip the two from the cap into the medicine cup. Leave the bottle standing and let go of everything.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_tablets_cap.run scripted --out out/scripted
python -m rdb_tablets_cap.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 4,800 steps (4 minutes at 20 Hz). Use the default layout; additional diagnostic variants are not the submitted trial.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 1,382 | 314 s |
| Dream-1 | 0.0 | 4,800 | 881 s |

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

Reset: Nothing is used up. Tip the cup, the cap and any strays back into the bottle and check it holds 30 whole tablets (replace chipped ones). With real ibuprofen, wear gloves for the reset and wipe the cap, the cup and the gripper pads afterwards. Stand the open bottle, the upside-down cap and the empty cup back on their three marks (tape rings on the table make this quick).

## Physics and reproduction

PhysX, 240 Hz physics/20 Hz control. Tablets move by gravity and contact. Bottle/cup colliders use 16 staves: bottle walls 4 mm (real about 1 mm; bore 6 mm narrower), cup 2.5 mm. Smooth meshes are display only. No threads, cap liner or child-resistant mechanism. Convex tablets, friction 0.35; internal dimensions and masses estimated. No chipping or dust. Full assumptions: REFERENCE-SPEC.json and TECHNICAL.md.

[TECHNICAL.md](TECHNICAL.md) contains the detailed build list, product/dimension sources, low-cost
reproduction options, physical scoring procedure, controller stages and measured limitations.
[REFERENCE-SPEC.json](REFERENCE-SPEC.json) preserves the full specification used for the recordings.
Only the card title, context and fidelity summary were shortened; instruction, setup, scoring, reset and run limit are unchanged.

## License and tests

Task code and procedural assets: MIT, see `LICENSE`. No manufacturer CAD, images or meshes are included.
Third-party runtime dependencies retain their own licenses, listed in TECHNICAL.md.
Run `python -m pytest -q tests` for the local checks. Dry runs are unit-level checks, not simulator evidence;
the scores above come from the recorded Isaac runs.
