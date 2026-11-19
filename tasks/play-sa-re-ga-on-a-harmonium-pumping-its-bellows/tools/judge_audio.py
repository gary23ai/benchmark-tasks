"""Judge a recorded (or synthesized) trial with the real-trial sound rule and print the score.

Usage:  python tools/judge_audio.py TRIAL.wav CALIBRATION.wav [--hold-s H]

The calibration recording must follow the README: every key C2 to D5 held 1.5 s each, 0.5 s apart, then 3 s of the
quiet room (recordings with only A3 to G4, 2 s each, as the first simulated runs wrote, are also read). The
harmonium's position and the release are judged from the video and are taken as satisfied here unless
``--moved-mm`` or ``--not-released`` say otherwise.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rdb_harmonium import audio, grading  # noqa: E402
from rdb_harmonium.grading import PhraseGrade  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial")
    parser.add_argument("calibration")
    parser.add_argument("--hold-s", type=float, default=grading.HOLD_S,
                        help="4.0, or the measured single-key sustain plus 1 s if that is longer")
    parser.add_argument("--moved-mm", type=float, default=0.)
    parser.add_argument("--not-released", action="store_true")
    args = parser.parse_args()
    grading._HOLD = int(round(max(args.hold_s, grading.HOLD_S) * grading.CONTROL_HZ))
    cal, rate = audio.read_wav(args.calibration)
    ref, noise, cross = audio.calibrate(cal, rate)
    trial, rate2 = audio.read_wav(args.trial)
    if rate2 != rate:
        raise SystemExit("trial and calibration must have the same sample rate")
    frames = audio.judge(trial, rate, ref, noise, cross)
    grade = PhraseGrade()
    position = (args.moved_mm / 1000., 0., 0.)
    result = None
    for f in frames:
        result = grade.update(f, position, np.eye(3), False, True)
    for _ in range(int(grading.RELEASE_S * grading.CONTROL_HZ)):
        result = grade.update((), position, np.eye(3), args.not_released, True)
    print(json.dumps({"score": result["score"], "longest_s": result["longest_s"], "joins_s": result["joins_s"],
                      "other_keys_s": result["other_keys_s"], "frames": len(frames),
                      "reference_db": {k: round(v, 1) for k, v in ref.items()}}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
