# Refresh a dull snap-off utility knife in a desktop blade disposal station

A worn snap-off knife tip drags or tears paper and card. Removing one pre-scored segment exposes the next manufactured edge without grinding or replacing the blade carrier. The OLFA DC-6 maker instructions (https://www.olfa.co.jp/en/products/510.html) show inserting the stainless sheath fully and pressing a lever, with the entire snapping process inside its collection case. The original desktop dock also stores knives and spare blades. This bounded task refreshes one dull segment for continued cutting; it does not claim sharpened-edge measurement or simulate cutting.

## Recorded results

| Policy | Score | Actions | Trials |
| --- | ---: | ---: | ---: |
| Scripted reference | 1.0 | 781 / 1600 | 2 |
| Dream-1 beta | 0 | 1600 / 1600 | 1 |

The submitted media contains the scripted and model recordings and the start image.

These are fixed-start native simulation trials with seed 0. Both policies used
the same instruction, mechanism, reset, control rate and budget. The current rubric was applied to both archived scripted trajectories. The recorded model never separated a segment, which is required for all nonzero tiers. The two deterministic scripted traces have the same input hash. No physical weighing trial was run.
The reference can inspect simulator body/contact state. Dream receives only
three 320 × 176 RGB images and 14 measured joint/gripper positions. The extra
side camera is for review. Guard changes and service session renewals are
recorded in the model summary; one model trial is not a success-rate estimate.

## Install and run

Prepare the pinned native runtime in [ENVIRONMENT.md](ENVIRONMENT.md), then run
these commands **from this task folder**, in that environment:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[test]'
python tools/verify_source.py
PYTHONPATH=. python -m pytest -q tests
PYTHONPATH=. python -m rdb_knife_refresh.run scripted --out runs/scripted-new
```

For a fresh model trial, install the model extra and provide your own private
credential file; the output directory must not already exist:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[dream]'
python tools/evaluate_dream.py --task refresh-a-snap-off-utility-knife --out runs/model-new --credential-file /path/to/private/credentials.env
```

The portable Dream runner keeps the stock SDK, inputs, guards and action budget.
Only package discovery and credential-path defaults differ from the original
recording entry point; no new model result is claimed for that packaging change.
One simulated trial lasts at most 80 seconds at 20 Hz.
The recorded model trial took 470.1 seconds of wall time
including initialization and inference; hardware and service latency vary.

## Specification and reset

The exact evaluated instruction, fixed starting layout, numeric rubric,
fidelity assumptions and reset are in [task.json](task.json).
[TECHNICAL.md](TECHNICAL.md) gives the mechanism and physical approximations;
[TASK-DESCRIPTION.md](TASK-DESCRIPTION.md) gives the real procedure and references.
The scripted runner scores every native physics substep and writes its original
`RESULT.json`, videos and grading inputs into its selected output directory.
`tools/verify_source.py` checks source identity, instruction and ordinary budget.


## How to grade a physical trial

Use a close side view of the guide and lever and a wider view of the box exterior, mat and bench. Keep ruler marks in frame. Photograph the tip before and after from the same angle against the ruler. Before the trial weigh the empty, closed box and tool separately. Use the same 0.01 g scale after the trial, without opening the box. Zero the trial for direct robot contact with the exposed segment, visible fragment escape, separation outside the disposal operation, damage to the fixture, or a component leaving the bench. A return after escape does not restore credit. Internal bounces are immaterial. 0.3: a visible lever press with the sheath in the guide, followed by withdrawal with exactly one segment missing (compare the two tip photos). 0.6: additionally the segment is accounted for inside the closed box. Inspect all exterior surfaces, mat and bench: no loose segment. The box must gain and the knife lose one segment mass (nominal 0.13 g; pre-weigh a matching spare with its protective tray tared); each change must agree within 0.02 g. Any confirmed loose segment outside voids the trial. 1.0: additionally the tool rests flat wholly on the mat, box upright within 25 mm of its start, hands released and exterior assembly at rest for one second. From side and overhead video check tilt (tool within 3 degrees of flat, box within 5 degrees of upright). Otherwise take the highest verified tier, or zero if none. If an exterior view is blocked and the other view cannot rule out escape, score zero. Missing evidence for a higher tier earns only the highest verified lower tier. The simulation uses its separation event and inventory geometry to represent the before/after checks.
