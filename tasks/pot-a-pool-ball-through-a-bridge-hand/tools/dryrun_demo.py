"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>] [--verbose]
See ``kinematic_world.py`` for what the stand-in world does and does not model.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, max_steps=None, clearance=None, verbose=False, variant="", idle=False, **world_kw):
    from yam_arena import ik
    from yam_arena.constants import READY

    from kinematic_world import World
    from rdb_pool_bridge import fixed_scene as fixture
    from rdb_pool_bridge.grading import PoolGrade
    from rdb_pool_bridge.scripted import PoolReference

    world = World(ik, clearance=clearance, **world_kw)
    reference = PoolReference(variant=variant)
    actions = reference.sequence()
    grade = PoolGrade()
    result = None
    for tick in range(1, (max_steps or fixture.MAX_STEPS) + 1):
        reference.observe(world.joint_pos(), world.state())
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        if idle:
            command = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        events = world.step(command)
        result = grade.update(world.sample(), events)
        if verbose and tick % 20 == 0:
            print(tick, reference.phases[-1][1] if reference.phases else "", result["score"], world.held,
                  np.round(world.tip, 3), {n: np.round(world.balls.position(n), 3).tolist() for n in ("cue", "one")})
        if result["finished"]:
            break
    report = dict(world.report)
    report.update(score=result["score"], steps=tick, legal=result["legal"], fouls=result["fouls"],
                  first_hit=result["first_hit"], potted=result["potted"], strike=result["strike"],
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), phases=reference.phases,
                  grips=reference.log["grips"], events=reference.log["events"],
                  servo_last=reference.log["servo"][-3:] if reference.log["servo"] else None)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-arena")
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs", *((args.yam_arena,) if args.yam_arena else ())):
        sys.path.insert(0, str(path))
    clearance = None
    if args.clearance:
        sys.path.insert(0, args.clearance)
        import clearance
    cases = (("nominal", {}),
             ("more sag, slow arm, 15 % slower stroke", dict(sag=(0., .04, 0., .09, .02, 0.), lag_s=.2, speed_scale=.85)),
             ("no sag, quick arm, 15 % faster stroke", dict(sag=(0.,) * 6, lag_s=.06, speed_scale=1.15)))
    for name, kw in cases:
        report = dry_run(clearance=clearance if name == "nominal" else None, verbose=args.verbose, **kw)
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items() if k != "phases"})
    for variant in ("one_hand", "no_slide"):
        report = dry_run(variant=variant)
        print("control", variant, "->", report["score"], report["legal"], report["strike"] and report["strike"].get("legal"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
