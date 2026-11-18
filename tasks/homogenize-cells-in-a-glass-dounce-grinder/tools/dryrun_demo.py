"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>]
See ``kinematic_world.py`` for what the stand-in world does and does not model.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, max_steps=None, clearance=None, verbose=False, variant=None, idle=False, **world_kw):
    from yam_arena import ik

    from kinematic_world import World
    from rdb_dounce import fixed_scene as F
    from rdb_dounce.grading import DounceGrade
    from rdb_dounce.scripted import DounceReference

    world = World(ik, clearance=clearance, **world_kw)
    reference = DounceReference(variant=variant)
    actions = reference.sequence()
    grade = DounceGrade()
    result = None
    for tick in range(1, (max_steps or F.MAX_STEPS) + 1):
        reference.observe(world.joint_pos(), world.state())
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        if idle:
            from yam_arena.constants import READY
            command = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        world.step(command)
        state = world.state()
        result = grade.update({**state, "tcp": [world.hand(arm)[0] for arm in F.ARMS]})
        world.grade = result
        if verbose and tick % 40 == 0:
            print(tick, result["score"], result["depth_mm"], result["lift_mm"], result["strokes"], result["good_strokes"])
        if grade.finished:
            break
    report = dict(world.report)
    report.update(score=result["score"], steps=tick, strokes=result["strokes"], good=result["good_strokes"],
                  foamed=result["foamed"], head_out=result["head_out"], failed=result["failed"],
                  peak_speed_mm_s=result["peak_speed_mm_s"], peak_lift_mm=result["peak_lift_mm"],
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), grasps=reference.log["grasps"],
                  events=reference.log["events"], stroke_log=grade.log)
    return report


def setup_paths(yam_arena=None):
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *((yam_arena,) if yam_arena else ())):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena")
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    setup_paths(args.yam_arena)
    clearance = None
    if args.clearance:
        sys.path.insert(0, args.clearance)
        import clearance
    cases = (("nominal", {}),
             ("more sag, slow arm, stiffer liquid (400 N s/m)", dict(sag=(0., .04, 0., .09, .02, 0.), lag_s=.2, drag=400.)),
             ("no sag, quick arm, thinner liquid (120 N s/m)", dict(sag=(0.,) * 6, lag_s=.06, drag=120.)),
             ("control: left hand never holds the tube", dict(variant="no_hold")),
             ("control: strokes at 45 mm/s", dict(variant="fast")),
             ("control: idle robot", dict(idle=True, max_steps=400)))
    for name, kw in cases:
        report = dry_run(clearance=clearance if name == "nominal" else None, verbose=args.verbose, **kw)
        report.pop("stroke_log")
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
