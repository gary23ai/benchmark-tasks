# Open a drawer fitted with a child-safety latch: hold it at the catch and press the latch down through the gap

Open a storage drawer past its child-safety latch to at least 150 mm, then leave it open with both hands clear.

## Instruction

> Open the drawer at least 150 mm, then let go of it. A child-safety latch inside stops it after about 38 mm: hold the drawer there, reach into the gap above the drawer front with the other hand and press the latch down, pull the drawer past the catch, then open it the rest of the way.

The instruction above is the exact string used in the recorded model run.

## Start scene

Sterilite 2301 drawer: 432 × 365 × 175 mm outside. Frame front at x = 390 mm, y = 0; frame clamped against a stop. Drawer starts shut. Safety 1st 48392 latch engages at about 38 mm of opening; the access gap is 32 mm.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

Opening ≥150 mm, no contact, and both gripper centres ≥50 mm from the unit for 2 s. Moving the frame more than 10 mm or breaking the latch invalidates the result.

## Coordinated operation

Hold the drawer at the catch, unload it slightly, depress the latch and pull past the hook before releasing the pressing hand. Pulling harder alone does not release the hook.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_child_latch.run scripted --out runs/scripted
python -m rdb_child_latch.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_child_latch/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 1,600 steps (80 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 532 / 1,600 | 117.75 s |
| Dream-1 | 0.0 | 1,600 / 1,600 | 283.17 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

The scorer is `rdb_child_latch/grading.py`; `task.json` states the same rules in words. It scores the state at
the end of the trial.

- **0** if the frame was shifted more than 10 mm on the table or the latch or catch is broken or off its screws
  (both apply to real trials; in the simulator the frame and catch are fixed and the pin cannot fail).
- **1.0**: open 150 mm or more, nothing of the robot touching the unit, its pull or the latch, and both gripper
  centre points at least 50 mm from the unit, for 2 s. The trial then ends.
- **0.8**: open 150 mm or more, a hand still on or near it.
- **0.6**: open 60 mm or more: the hook is past the catch, which it can pass only pressed down.
- **0.25**: open 30 mm or more: pulled out to the catch (it stops the drawer at 38 mm).
- **0** otherwise.

Outcomes: nothing done, 0; the drawer pulled, however hard, 0.25 (it stops at 38 mm); the latch pressed and let
go before pulling, 0.25 (the spring puts the hook back up); pressed and pulled past the catch, 0.6; opened to
150 mm with a hand still on it, 0.8; opened and let go, 1.0; opened and pushed back in, the score for where it
ends.

### Scoring a real trial

1. Tape the frame's outline on the table and a ruler beside the drawer, zero at the frame's front face. Film from
   above and from the left front.
2. At the end, read the opening at the outer face of the drawer front.
3. From the video, check that for the last 2 s nothing of the robot touches the unit and both grippers are 5 cm
   or more from it.
4. Check the frame against its taped outline (10 mm) and that the latch and catch are whole and on their screws.

## Reset

Push the drawer shut: the hook rides up the catch's rounded front edge and the latch engages again. Check it: pulled by its lip, the drawer must stop at 38 mm (+/- 3 mm). Arms at their ready pose, grippers open. Nothing is consumed. Before the first trial and after every 20: tug the drawer firmly at the catch (the maker's check: it must hold), and with a kitchen scale under a pencil's rounded end pressed on the latch arm 21 mm behind its pivot, check that it frees the hook at 3 to 7 N; also check that pressing the drawer front down by hand does not free the hook.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

The hook and catch block and release the drawer through contact. A torsion spring replaces the latch’s moulded plastic spring. Drawer friction, internal heights and spring stiffness are estimates. The runner has no play; no software rule releases the latch.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_child_latch/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_child_latch/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_child_latch/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_child_latch/scripted.py` | Feasibility controller |
| `rdb_child_latch/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.
