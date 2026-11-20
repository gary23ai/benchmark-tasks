# Remove a stroller rear wheel for axle and hub servicing

Parents and repair workers remove pushchair wheels to access the axle and hub during maintenance. The Britax B-AGILE owner guide (printed p7, https://csus.britax.com/media/7460/p590000b_b-agile-ug_low-res-web.pdf) documents holding the bottom release lever while drawing the rear wheel away. This bounded task performs that removal and stable workshop storage; it does not claim cleaning, repair, reinstallation or roadworthiness inspection.

## Recorded results

| Policy | Score | Actions | Trials |
| --- | ---: | ---: | ---: |
| Scripted reference | 1.0 | 604 / 700 | 2 |
| Dream-1 beta | 0 | 700 / 700 | 1 |

[Initial scene](../../uploads/remove-a-stroller-rear-wheel-for-servicing/setup.png) · [Full-score demo](../../uploads/remove-a-stroller-rear-wheel-for-servicing/demo.mp4) · [Full model run](../../uploads/remove-a-stroller-rear-wheel-for-servicing/model_run.mp4) · [Validation](validation.json)

These are fixed-start native simulation trials with seed 0. Both policies used
the identical instruction, task source, scorer, reset, control rate and budget.
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
PYTHONPATH=. python -m rdb_stroller_remove.run scripted --out runs/scripted-new
```

For a fresh model trial, install the model extra and provide your own private
credential file; the output directory must not already exist:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[dream]'
python tools/evaluate_dream.py --task remove-a-stroller-rear-wheel-for-servicing --out runs/model-new --credential-file /path/to/private/credentials.env
```

The portable Dream runner keeps the stock SDK, inputs, guards and action budget.
Only package discovery and credential-path defaults differ from the original
recording entry point; no new model result is claimed for that packaging change.
One simulated trial lasts at most 35 seconds at 20 Hz.
The recorded model trial took 238.3 seconds of wall time
including initialization and inference; hardware and service latency vary.

## Specification and reset

The exact evaluated instruction, fixed starting layout, numeric rubric,
fidelity assumptions and reset are in [task.json](task.json).
[TECHNICAL.md](TECHNICAL.md) gives the mechanism and physical approximations;
[TASK-DESCRIPTION.md](TASK-DESCRIPTION.md) gives the real procedure and references.
The scripted runner scores every native physics substep and writes its original
`RESULT.json`, videos and grading inputs into its selected output directory.
`tools/verify_source.py` checks source identity, instruction and ordinary budget.

The evidence folder contains clearly marked summaries and hashes of the
original results and independently completed audits. Full raw input archives
are retained separately, rather than represented by truncated traces here.
The initial screenshot and full uncut videos are packaged in `uploads/remove-a-stroller-rear-wheel-for-servicing`
beside `tasks/` in the delivery ZIP. Task and model source are unchanged by prose
cleanup. No official upload, media ID or acceptance is claimed in this branch.
