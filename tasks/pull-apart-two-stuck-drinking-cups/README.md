# Pull apart two drinking cups that are stuck one inside the other

Separate two nested plastic cups and leave both standing upright on the table, released and still.

## Instruction

> The two cups are stuck together. Get them apart and leave both standing upright on the table.

The instruction above is the exact string used in the recorded model run.

## Start scene

Two IKEA KALAS tumblers, article 004.613.79: rim 80 mm, base 66 mm, height 81 mm, 30 g each. Outer cup base at (275, +35, 0) mm; inner cup on the same axis, 17.9 mm higher. The simulated fit releases above 6 N or 0.3 N·m.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

Neither cup remains inside the other. Both stand within 10° of vertical, released, still and ≥50 mm from the grippers for 2 s. Throwing, dropping or losing either cup invalidates the trial.

## Coordinated operation

Hold the outer cup while pulling the inner cup by its exposed rim. A one-handed lift first demonstrates that the pair moves together. Regrips and rocking are allowed; the score depends on separation and the final placement.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_nested_cups.run scripted --out runs/scripted
python -m rdb_nested_cups.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_nested_cups/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 2,400 steps (120 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 1,342 / 2,400 | 316.99 s |
| Dream-1 | 0.0 | 2,400 / 2,400 | 473.52 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

The trial is judged from the video and the final state. Definitions:

- **Apart**: no part of either cup is inside the other. (Scorer: no point of one cup's axis lies within the
  other cup's outline.)
- **Standing**: the cup rests on its base on the table top, within 10 degrees of upright, not moving, no
  gripper touching it and the centre point of each gripper (midway between the fingertips) at least 5 cm
  from its surface, throughout the last 2 s. (Scorer: base centre between 3 mm below and 5 mm above the
  table top; under 1 cm/s and 0.15 rad/s; the 5 cm is measured to a cylinder of the rim's radius round
  the cup's axis, so it is never more lenient than the real surface.)
- **Thrown or dropped**: at any moment the cup's lowest point is more than 5 cm above the table, it is
  moving (scorer: faster than 0.2 m/s) and no gripper centre point is within 10 cm of the cup's centre.
- **Off the table**: the cup's centre passes a table edge, or its base is more than 3 cm below the table top.

| Score | Outcome |
|---:|---|
| 0 | Cups not apart at the end (still stuck; loosened but one still in the other; the stuck pair knocked over); or a cup was thrown, dropped or left the table at any time; or (real trial) a cup ends cracked, creased or permanently dented |
| 0.5 | Apart, neither cup standing (both still held; both on their sides) |
| 0.75 | Apart, one cup standing (the other still held, lying down, upside down or leaning) |
| 1.0 | Apart, both cups standing |

The trial ends at 2,400 steps, or as soon as 1.0 is reached. Where the cups stand, their spacing, and which
arm did what do not matter.

### Scoring a real trial

1. Film the whole trial from the front or side with the table edge and a 5 cm height marker in view.
2. From the video: did a cup ever fly or fall through the air from more than 5 cm up with no gripper within
   10 cm, or leave the table top? If so, score 0.
3. When the arms stop or time runs out, photograph the table from above and from the side. Inspect both cups
   by hand afterwards: a crack, crease or permanent dent scores 0.
4. From the photos: is any part of one cup inside the other? If so, score 0. Otherwise 0.5.
5. For each cup: is it on its base on the table, upright (a level or protractor against its side: within
   10 degrees), still for the last 2 s of the video, with no gripper touching it and both grippers' centre
   points at least 5 cm from it (tape measure)? Add 0.25 for each.

## Reset

Use dry cups. Stand the blue cup on its mark, set the yellow cup inside it and press it straight down with 10 N for 2 s (stand the pair on a kitchen scale, zero it, press until it reads 1.0 kg). Check that lifting the yellow cup by its rim lifts both cups. Replace any cup that is cracked or creased. Arms at their ready pose, grippers open. In simulation: seed 0, zero object jitter; both cups are returned to their authored poses with zero velocity, the bond between them is whole again and the scene settles for 0.5 s; the scorer's remembered events start fresh.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

A breakable fixed joint represents the stuck fit; it releases abruptly and does not re-form. The exposed inner rim is thickened inward for collision, and the lower interior is collision-solid. Cup deformation and suction are absent. The 6 N release force is a model setting, not a measurement on the purchased cups.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_nested_cups/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_nested_cups/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_nested_cups/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_nested_cups/scripted.py` | Feasibility controller |
| `rdb_nested_cups/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.
