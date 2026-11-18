"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

``world.py`` stands in for the simulator: arms with a lag, a drawer that moves only with the hand that holds
its pull and stops at the catch while the latch is up, a latch arm that goes down only under a fingertip. The
demonstration gets the same state it gets in the simulator and ``grading.py`` scores every step. This checks
reach, joint limits, clearances, control flow and the scoring; it says nothing about contact forces.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


TIP_LINKS = ("tip_left", "tip_right")
BODY_LINKS = ("gripper", "link5", "link4", "link3", "link2")
THIN = 6                 # use every sixth point of the arms' hulls: fast enough, a few mm coarser


def signed_gap(cloud, boxes):
    """Smallest distance from any point of ``cloud`` to any of the boxes (low, high); negative inside a box."""
    best = np.inf
    for low, high in boxes:
        d = np.maximum(np.asarray(low) - cloud, cloud - np.asarray(high))
        outside = np.linalg.norm(np.maximum(d, 0.), axis=1)
        best = min(best, float(np.where(outside > 0., outside, d.max(axis=1)).min()))
    return best


def lowest(report, key, value, tick):
    if value < report.get(key + "_mm", np.inf):
        report[key + "_mm"], report[key + "_tick"] = value, tick


def dry_run(variant="full", *, max_steps=None, clearance=None, verbose=False, **world_options):
    from yam_arena import ik

    from build_scene import catch_box, drawer_boxes, frame_boxes
    from rdb_child_latch import fixed_scene as fixture
    from rdb_child_latch import layout as L
    from rdb_child_latch.grading import DrawerGrade
    from rdb_child_latch.scripted import LatchReference
    from world import World

    world = World(**world_options)
    reference = LatchReference(variant)
    actions = reference.sequence()
    grade = DrawerGrade()
    joints = world.joints.copy()
    # Gaps from the arms' collision hulls (dry-run estimates): between the two arms; from any arm part to the
    # frame, the catch and the table; from everything but the fingertips to the drawer and the latch; from the
    # pressing fingertips to the drawer (front, plate) - they should touch only the latch arm; from the pulling
    # fingertips to every drawer box but the pull - they should touch only the pull.
    report = {"worst_limit_margin_deg": 180., "peak_score": 0., "peak_latch_deg": 0.}
    result, steps = None, max_steps or fixture.MAX_STEPS
    fixed = [(np.asarray(low), np.asarray(high)) for _, low, high in frame_boxes()]
    _, low, high = catch_box()
    fixed.append((np.asarray(low), np.asarray(high)))
    names = [name for name, _, _ in drawer_boxes()]
    for tick in range(1, steps + 1):
        reference.observe(joints, world.state())
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        joints = world.step(command)
        report["peak_latch_deg"] = max(report["peak_latch_deg"], float(np.degrees(world.angle)))
        if clearance is not None and tick % 2 == 0:
            clouds = [clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm], links=BODY_LINKS)
                      for i, arm in enumerate(fixture.ARMS)]
            tips = [clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm], links=TIP_LINKS)
                    for i, arm in enumerate(fixture.ARMS)]
            clouds, tips = [c[::THIN] for c in clouds], [t[::2] for t in tips]
            lowest(report, "min_arm_gap", 1000 * clearance.arm_gap(
                np.concatenate((clouds[0], tips[0])), np.concatenate((clouds[1], tips[1]))), tick)
            moving = world.boxes()
            drawer = moving[:len(names)]
            for arm, cloud, tip in zip(fixture.ARMS, clouds, tips, strict=True):
                lowest(report, "min_table_gap", 1000 * float(min(cloud[:, 2].min(), tip[:, 2].min())), tick)
                lowest(report, "min_frame_or_catch_gap", 1000 * signed_gap(np.concatenate((cloud, tip)), fixed), tick)
                lowest(report, "min_hand_body_gap", 1000 * signed_gap(cloud, moving), tick)
            lowest(report, "press_tips_to_drawer", 1000 * signed_gap(tips[0], drawer), tick)
            lowest(report, "pull_tips_to_drawer_but_pull",
                   1000 * signed_gap(tips[1], [b for b, n in zip(drawer, names) if n != "pull"]), tick)
        result = grade.update(*world.grade_inputs())
        report["peak_score"] = max(report["peak_score"], result["score"])
        if verbose and tick % 20 == 0:
            print(tick, result["score"], round(1000 * world.open, 1), round(float(np.degrees(world.angle)), 1),
                  world.gripped, reference.phases[-1][1])
        if result["score"] == 1.:
            break
    report.update(score=result["score"], steps=tick, open_mm=result["open_mm"], hands_clear=result["hands_clear"],
                  gripper_clearance_mm=result["gripper_clearance_mm"], fingertip_below_bottomed_arm_mm=round(1000 * world.bottomed, 2),
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), phases=len(reference.phases),
                  tries=[(t["attempt"], t["pressed"], t["past"]) for t in reference.log["press_tries"]])
    return report, reference


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
    cases = (("full, nominal", "full", {}),
             ("full: soft slow arm, pressing hand 4 mm toward the pin and 6 mm high, pulling hand 4 mm short", "full",
              dict(lag_s=.25, press_offset=(-.004, 0., .006), pull_offset=.004)),
             ("full: stiff arm, pressing hand 4 mm toward the frame and 5 mm low, hook frees only at 8 degrees", "full",
              dict(lag_s=.08, press_offset=(.004, .005, -.005), hold_angle=np.radians(8.))),
             ("full: both arms sag (elbow 0.03 rad), which the joint readings show", "full", dict(sag=.03)),
             ("control: pull only", "no_press", {}),
             ("control: press, lift, then pull", "early_release", {}))
    for name, variant, options in cases:
        report, _ = dry_run(variant, clearance=clearance if name == "full, nominal" else None, verbose=args.verbose, **options)
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
