# Play three notes on a hand-pumped harmonium

One YAM arm pumps the rear bellows while the other plays the marked Sa, Re and Ga keys in order. Each note must last four seconds, with no audible gap between notes. Both arms then withdraw and all keys remain up for one second.

## Recorded result

Native run 8: scripted **1.0** in **464 actions** (23.2 s of simulation, 134 s wall time); Dream-1 **0.0** in **2,400 actions** (555 s wall time). The script held the three notes for 4.7, 4.7 and 4.4 seconds, joined them legato, pressed no stray key and released both hands. These are one fixed trial per policy, not hardware success rates.

Both policies use seed 0, no object jitter, 20 Hz control and the same scene, air rule and simulation scorer. Dream-1 receives 320 x 176 overhead and two wrist RGB images, joint and gripper positions. It receives no audio, air state or review side camera. The reference script reads simulator state but moves the board and keys only through robot contact; it does not teleport or attach objects.

## Install and run

Use the Inspect Robots dual-arm YAM Isaac Lab-Arena runtime with Isaac Sim 6.1. The workstation and YAM assets are provided by that runtime; they are not bundled here. Install the task next to the runtime:

```bash
pip install -e .
python -m rdb_harmonium.run scripted --out runs/scripted
python -m rdb_harmonium.run dream --out runs/dream
python -m pytest -q tests
```

Dream-1 uses the runtime's Dream Labs credentials. A trial permits at most 2,400 actions (120 s at 20 Hz). RESULT.json contains the score, steps, hashes and final state. The media directory contains the initial screenshot, overhead/side videos, scorer trace, synthesized sound and calibration recording.

## Score and reset

The primary score progresses through 0, 0.25, 0.5, 0.75, 0.85 and 1.0. Full score requires all three sustained notes, legato joins, no prolonged stray note, limited case motion, and complete release. Moving the case over 30 mm or tipping it over 5 degrees latches a zero. Numeric thresholds and the real microphone-based rule are in task.json and TECHNICAL.md.

For a real trial, calibrate every key from C2 to D5 and record room noise before running. Analyze the recording with:

```bash
python tools/judge_audio.py media/sound.wav media/calibration.wav
```

Reset by emptying the air reservoir, allowing the bellows board to spring open, releasing all keys, setting only the main stop out 20 mm, replacing the case on its marks and returning the arms to ready. Nothing is consumed.

## Evidence and approximations

validation.json records the native results and source hashes. The simulation models air and reed response by stated rules, not fluid dynamics or acoustic simulation. Sizes and key range come from the listed instrument; force, reservoir and flow parameters are estimates. TECHNICAL.md retains the dimensions, parameter reasoning, audio-judge tests and development results. The real-audio helper was improved after run 8; the physical simulation and primary scorer were unchanged, and the earlier helper is retained in evidence/.

## License

Task code and generated instrument assets are MIT licensed; see LICENSE. No manufacturer CAD or audio samples are bundled. YAM and workstation assets retain the licenses supplied with their runtime.
