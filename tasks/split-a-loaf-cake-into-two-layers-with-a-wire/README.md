# Split a loaf cake into two layers with a cutting wire held taut between both hands

Use a two-handled wire to split a loaf cake horizontally at the 30 mm mark. Leave the top layer on the bottom layer and put both handles back on the table.

## Instruction

> Split the loaf cake into two layers of equal height with the cutting wire. Pick up the wire by its two handles, hold it taut and draw it level through the cake along its length, half way up: 30 mm above the mat the cake stands on, where its sides are scored. Do not touch the cake with the grippers. Leave the cake where it is with the top layer still lying on the bottom one, and put the wire down on the table when you have finished.

The instruction above is the exact string used in the recorded model run.

## Start scene

Cake: Entenmann’s All Butter Loaf Cake, 326 g, modelled 185 × 90 × 60 mm, centre (400, 0) mm. Stand 250 × 150 × 40 mm with silicone mat. Fante’s 2552 wire: 305 mm long, estimated 0.5 mm diameter, two 65 × 12 mm handles. Black film improves contrast.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

Cut ≥99% of the cake plan within ±5 mm of the 30 mm line, without touching the cake or spoiling another section. Cake and top layer remain within 10 mm and 5° of their intended poses. Handles are on the table and both open grippers are ≥50 mm clear for 2 s.

## Coordinated operation

Grip both handles, tension the wire and advance both ends level through the cake. Pulling too hard drags the cake; moving the ends unequally changes the cut height. Touching the cake with an arm or gripper scores zero.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_cake_wire.run scripted --out runs/scripted
python -m rdb_cake_wire.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_cake_wire/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 1,600 steps (80 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 879 / 1,600 | 286.23 s |
| Dream-1 | 0.0 | 1,600 / 1,600 | 522.20 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

Judged at the end of the trial, except "touched", which is kept from the moment it happens.

| Term | In simulation | In a real trial |
|---|---|---|
| Cut length | Cells of the two outer rows of the plan grid cut within 5 mm of the line, as a length | Ruler along each long side of the cake: length of cut within 5 mm of the scored line |
| Cut through | 99 percent of the plan cut within 5 mm of the line | The top layer lifts off whole; a ruler against the cut faces reads 25 to 35 mm for both layers at the four corners and the middle of each long side |
| Spoiled | More than 5 percent of the plan cut at any other height | A second cut, or a stretch of cut more than 5 mm off the line, over more than 8 cm2; a layer torn or broken |
| Touched | Any arm link pressed on the cake with more than 0.05 N at any time | From the video: a gripper or arm touched the cake; or a finger mark on the crust |
| In place | Cake within 10 mm and 5 degrees of its start; top layer within 10 mm, 5 degrees and 5 mm of height of its seat | Cake against its chalk marks; top layer lying on the bottom one, edges within 10 mm |
| Put down | Both handles' middles no more than 11 mm above the table top and not over the stand; both grippers at least 80 percent open; each gripper centre point at least 5 cm from both handles and the cake; nothing faster than 20 mm/s; for 2 s | From the video and a tape measure |

| Score | Outcome |
|---:|---|
| 0 | The cut runs less than 60 mm along either long side, or the cake was touched |
| 0.25 | The cut runs at least 60 mm along both long sides, but the cake is not cut through, or it is spoiled |
| 0.5 | Cut through and not spoiled, but the cake was dragged or turned, or the top layer was pulled off its seat |
| 0.75 | Cut through, cake and top layer in place, but the wire is not put down |
| 1.0 | Cut through, in place, wire put down |

## Reset

One cake per trial (about 5 to 6 US dollars); a cut cake cannot be used again, because a second cut at the same height meets no crumb. Take the cut cake away, brush the crumbs off the mat and the film, take a fresh cake out of its box and tray, stand it on the chalk marks and score the line 30 mm above the mat along its sides and ends with a knife held flat on a 30 mm block (2 minutes). Use cakes of one batch where possible and weigh each one; they vary by a few grams and millimetres. Food hygiene: only the wire touches the cake, and the grippers touch only the wooden handles; wash the wire between trials and do not eat the cakes. If a failed trial put a gripper into the cake, wipe the finger pads clean of crumbs and butter before the next trial (thin disposable finger covers make this quicker). Lay the wire out with the handles on their marks. Arms at their ready pose, grippers open. In simulation: seed 0, zero object jitter; both layers and both handles are returned to their authored poses with zero velocity, the record of the cut is cleared, the wire is laid slack between the handles, and the scorer starts fresh.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

Cutting is a stated material-response rule over a 2.5 mm grid, not a PhysX fracture simulation. Two rigid layers cannot crumble or compress. The wire is a massless spring chain, drawn thicker for visibility; its contact with robot links does not feed force back to the arms. Resistance, friction and several dimensions are estimates; loose-handle forces are reduced for numerical stability.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_cake_wire/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_cake_wire/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_cake_wire/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_cake_wire/scripted.py` | Feasibility controller |
| `rdb_cake_wire/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.
