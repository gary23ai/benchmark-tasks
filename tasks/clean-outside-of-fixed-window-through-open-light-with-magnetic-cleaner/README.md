# Clean the outside of a fixed window pane from indoors with a double-sided magnetic window cleaner, fitted through the open light beside it

Fit one half of a magnetic cleaner on each face of a fixed window pane. Move the indoor half to wipe the outdoor marker band, then leave the pair attached without either hand touching it.

## Instruction

> You are indoors. The black mark is on the outside of the fixed window pane, too far along to reach from the open window beside it. Clean it off from the inside with the magnetic window cleaner, then let go and leave the cleaner holding on the glass.

The instruction above is the exact string used in the recorded model run.

## Start scene

Wall: 1280 × 560 × 12 mm, indoor face x = 400 mm. Open light: 430 × 320 mm. Fixed pane: 550 × 320 × 4 mm. Outdoor marker band: 150 × 30 mm, y = +410 to +560 mm, z = 280 to 310 mm. Two cleaner halves start indoors, pad down.

Coordinates are millimetres: x forward from the line through the arm bases, y to the robot’s left, z above the table. The two arm bases are at (0, +305) and (0, −305) mm. Both arms start at the standard ready pose with grippers open. Layout and physical parameters are fixed between the demonstration and model trial.

[task.json](task.json) gives the complete start poses, dimensions and materials. [TECHNICAL.md](TECHNICAL.md) lists product sources, estimated parameters, the build procedure and asset licences.

## Successful outcome

At least 90% of the band is clean. The halves remain paired, flat on opposite faces, and still for 2 s; both grippers are at least 80 mm from the handles. Reaching through the wall outside the opening invalidates the trial.

## Coordinated operation

One hand positions the outdoor half through the opening while the other aligns the indoor half. Once coupled, the indoor hand drives the outdoor pad magnetically. Direct wiping by a held outdoor pad does not earn cleaning credit.

The scorer evaluates the physical outcome. It does not award points for using a particular arm or for following the reference script.

## Run and inspect the result

Use Linux with an NVIDIA RTX-capable GPU and the pinned [YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1): Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena 0.3.1. Follow that runtime’s installation instructions and source its `yam.env` before running these commands from this task folder. Isaac Sim is an external dependency, not part of this package.

```sh
uv pip install 'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0'
uv pip install 'dreamlabs-client==1.0.14' 'inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
uv pip install 'numpy>=1.26' 'scipy>=1.11' 'pillow>=10' 'imageio[ffmpeg]'
uv pip install --no-deps .
python -m rdb_window_wall.run scripted --out runs/scripted
python -m rdb_window_wall.run dream --out runs/dream
```

Configure Dream Labs credentials privately on the run host before running the `dream` trial. Both commands use Inspect Robots and the same scorer in `rdb_window_wall/grading.py`. The scripted controller may read simulator state; it is a feasibility reference, not a policy with the same information as Dream-1.

Outputs: `RESULT.json` (score and diagnostics), `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4` and `media/grader-trace.jsonl`. Read the final score in `RESULT.json`; the trace explains when each score changed. Dream-1 receives the overhead and two wrist RGB images at 320 × 176 plus standard joint/gripper position readings. The side camera is recorded for review only. Grader state and diagnostic contact forces are not model observations.

Fixed seed 0, zero object jitter, 20 Hz control and 240 Hz physics. Run limit: 2,400 steps (120 simulated seconds). Wall time includes loading and recording and is longer than robot time.

| Recorded trial | Score | Control steps | Wall time |
|---|---:|---:|---:|
| Scripted demonstration | 1.0 | 1,174 / 2,400 | 271.16 s |
| Dream-1 | 0.0 | 2,400 / 2,400 | 415.65 s |

These are individual recorded simulation trials. [validation.json](validation.json) records the run settings, scores and evidence scope. No hardware trial is reported.

Tests without launching Isaac Sim, using the installed YAM Python environment:

```sh
uv pip install pytest
python -m pytest tests -q
```


## Scoring

The trial is judged from the video, photos of the glass and a ruler. Definitions:

- **On the glass**: a half's pad lies flat against a face of the fixed pane. (Scorer: within 5 degrees; pad
  face between 1.5 mm into and 2 mm off the glass face; pad centre over the glass.)
- **Paired**: the two halves are on the glass on opposite faces and their outlines line up through it,
  centres within 15 mm. Which half is on which face does not matter.
- **Let go**: no part of the robot touches either half (scorer: contact force above 0.02 N) and the centre
  point of each gripper, midway between its fingertips, is at least 8 cm from the middle of each handle bar.
- **Cleaned area**: the band is 150 x 30 mm, 4,500 mm2, and it is scored over its whole area, not along a
  line. Real trial: the part of the band where no marker remains (see below). Simulation: the scorer divides
  the band into 180 squares of 5 mm (6 rows of 30); a square counts once the pad of the half on the band's
  face has slid over its centre, moving at 5 mm/s or more, while the halves were paired and no part of the
  robot touched that half. A stroke that covers only part of the band's height gets only that part.
- **Out of bounds**: the centre point of a gripper passes the plane of the wall anywhere but through the open
  light, that is over the top of the wall or round its ends. It scores 0 for the rest of the trial.

| Score | Outcome |
|---:|---|
| 0 | A gripper went out of bounds; or the halves were never paired: nothing done, a half left on the table, halves stuck to each other without the glass between them, both on one face, caught one magnet off; or (real trial) the glass cracked |
| 0.25 | Paired at some moment, but at the end not paired, let go and still for the last 2 s: still held, slipped out of line, or a half fell |
| 0.5 | At the end paired, let go and still (under 1 cm/s) for 2 s; under 50 % of the band's area cleaned |
| 0.75 | The same, with at least 50 % of the band's area (2,250 mm2) cleaned |
| 1.0 | The same, with at least 90 % of the band's area (4,050 mm2) cleaned |

The trial ends at 2,400 steps, or as soon as 1.0 is reached. Where on the pane the halves are paired and
left, which arm did what, and how many strokes were made do not matter. A half that falls outdoors lies on
the table beyond the wall; the robot may fetch it back in through the opening, if it can, and fit it again.

### Scoring a real trial

1. Film the whole trial from outdoors, beyond the fixed light's far end, so that both faces of the pane, the
   opening and the band are in view. Photograph the band before the trial, square on, with a ruler held
   under it.
2. From the video, note whether a gripper ever went over the wall or round its end (score 0), each stretch
   of time in which a gripper was touching the half on the outdoor face, and whether the two halves were ever
   on opposite faces and lined up.
3. When the arms stop or time runs out: are the halves on opposite faces, flat on the glass, outlines lined
   up within 15 mm (ruler through the glass), nothing touching them, both grippers at least 8 cm from the
   handles, and has that been so for the last 2 s of the video? If not, score 0.25 if they were ever paired,
   else 0.
4. Otherwise slide the cleaner off the band, photograph the band square on with the ruler, and measure the
   area where no marker remains: lay a 5 mm grid over the photo (6 rows of 30 squares over the 150 x 30 mm
   band) and count the squares that are clean at their centre. Take off any squares that the video shows
   were wiped by a gripper, or by the outside half while a gripper was touching it. Under 90 squares: 0.5.
   90 or more: 0.75. 162 or more: 1.0.
5. A cracked pane scores 0.

## Reset

Take both halves off the glass (slide one sideways off the other; do not let them snap together without the spacer); fetch a half that fell outdoors. Wipe the glass, redraw the 150 x 30 mm marker band on the outdoor face, 350 to 500 mm from the stile and 280 to 310 mm above the table, and let it dry for a minute. Lay each half pad down on its mark indoors, handle up, handle bar parallel to the wall, the blue half's apex towards the wall and the white half's towards the robot. Shake out or change a pad that is loaded with marker. Arms at their ready pose, grippers open. In simulation: seed 0, zero object jitter; both halves are returned to their authored poses with zero velocity, the scene settles for 0.5 s and the scorer's memory (paired, out of bounds, cleaned squares) starts fresh.

For a simulation reset, start a new run with a fresh output directory. The adapter reloads the fixed scene, resets velocities and joints, and clears the scorer’s event history. The physical preparation and replacement checks above apply before every real trial; estimated properties must be measured on the purchased objects before hardware replication.

## Simulation model

Magnetic forces and torques act at every physics step; the halves are not welded to the glass. Magnet strength and pad friction are estimates. Cleaning is a 5 mm grid rule, the glass cannot break, and an explicit gripper-boundary rule prevents reaching around the wall. Decorative window details have no collision.

See the full `fidelity` field in [task.json](task.json) and [TECHNICAL.md](TECHNICAL.md) for the collision geometry and parameter sources. The demonstration establishes solvability in this simulation; it does not measure correspondence to real hardware.

## Files and licence

| File | Purpose |
|---|---|
| `task.json` | Task definition and submission fields |
| `rdb_window_wall/fixed_scene.py` | Fixed instruction, layout and run settings |
| `rdb_window_wall/adapter.py` | Inspect Robots embodiment and sensor/scorer integration |
| `rdb_window_wall/run.py` | Scripted and Dream-1 trial entry point |
| `rdb_window_wall/scripted.py` | Feasibility controller |
| `rdb_window_wall/grading.py` | Scoring implementation |
| `validation.json` | Recorded scores and settings |
| `TECHNICAL.md` | Dimensions, sources, modelling details and real-trial preparation |
| `tests/` | Existing scene, controller and scoring tests |

Task-specific source and generated geometry are provided under [MIT](LICENSE). No third-party product meshes or textures are bundled. Robot, station and simulator dependencies retain their upstream licences; see the asset list in [TECHNICAL.md](TECHNICAL.md).

The submission card uses short title/setup/fidelity fields to meet the board’s length limits. [REFERENCE-SPEC.json](REFERENCE-SPEC.json) retains the full specification used by the recorded trials; [TECHNICAL.md](TECHNICAL.md) explains its dimensions and modelling assumptions. The instruction and scoring are unchanged.
