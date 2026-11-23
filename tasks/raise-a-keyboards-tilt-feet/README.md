# Unfold both rear keyboard feet and turn the keyboard over without collapsing either support

With the keyboard already underside-up after cleaning, unfold both rear tilt feet, then turn it keys-up and leave it resting securely on both extended feet and its front pads. Leave both hands clear.

Each rear foot folds independently. After the keyboard is turned keys-up, one foot can tuck under even though the keyboard looks tilted from the front. Inspect both rear corners after release. The Logitech K270 is a real example of a full-size keyboard with adjustable feet.

## Real-trial grading

Start underside-up. Score 0.3 when one rear foot has opened past 110 degrees from folded, 0.6 when both have, and 0 otherwise. Full credit requires both still open past 115 degrees after turnover. With keys up and both hands and the notebook clear, the keyboard must sit entirely on the desk, tilted 4–14 degrees front to back. Both rear toes and both front pads must be within 1 mm of the desk. View each rear corner separately against a 1 mm reference, then confirm the two front pads. Hold this released pose for 1 s without sliding more than 2 mm from the start of the hold or rotating more than 2 degrees. A folded-under foot or a visible gap under one support caps the score at 0.6. Falling off the desk voids the trial.

## Recorded evidence

The two archived demonstrations were rescored from native pose/contact samples. Dream never opened either foot; the recorded cumulative opening flags rule out partial or full credit under the revised rubric. The original videos finish at step 1411. The revised full-score interval first completes at native sample 16912 (action 1410) and stays satisfied through the end. The Dream recording uses all 1800 actions. No fresh policy inference is claimed.

## Install and run

Prepare the pinned native runtime in [ENVIRONMENT.md](ENVIRONMENT.md), then run
these commands **from this task folder**, in that environment:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[test]'
PYTHONPATH=. python -m pytest -q tests
PYTHONPATH=. python -m rdb_keyboard_feet.run scripted --out runs/scripted-new
```

For a fresh model trial, install the model extra and provide your own private
credential file; the output directory must not already exist:

```sh
uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[dream]'
python tools/evaluate_dream.py --task raise-a-keyboards-tilt-feet --out runs/model-new --credential-file /path/to/private/credentials.env
```

The portable Dream runner keeps the stock SDK, inputs, guards and action budget.
Only package discovery and credential-path defaults differ from the original
recording entry point; no new model result is claimed for that packaging change.
One simulated trial lasts at most 90 seconds at 20 Hz.
The recorded model trial took 250.0 seconds of wall time
including initialization and inference; hardware and service latency vary.



## Assets

Original procedural keyboard and notebook geometry is provided under the included MIT license. Robot meshes and the stock station come from the external pinned YAM runtime. See ENVIRONMENT.md for exact dependencies.
