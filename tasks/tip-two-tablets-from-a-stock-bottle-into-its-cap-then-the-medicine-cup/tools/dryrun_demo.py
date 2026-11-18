"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

The arms follow their joint commands with a first-order lag. A gripper that closes on the bottle or the cap
within 10 mm of its axis carries it rigidly (and counts as holding it for the scorer); opened, it leaves it standing where it is. Tablets follow a toy
rule, not physics: tipped past ``spill_deg`` the bottle lets tablets out in the given burst pattern, each
spending 0.3 s in the neck and landing under the lip; a cap tipped past ``cap_spill_deg`` (its wall then slopes down to the rim) lets
one tablet go every ``cap_gap_s`` from its low rim, into whatever lies under that point. This checks reach, joint limits, arm clearances,
the control flow (including the too-many and too-few branches) and the scorer. It says nothing about finger
contact or about how tablets really flow.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def dry_run(*, bursts=(1, 1, 1, 1, 1, 1), spill_deg=120., arm_lag_s=.12, joint_bias=(0., 0., 0., 0., 0., 0.),
            miss_mm=0., cap_spill_deg=105., cap_gap_s=.4, cap_burst=1, max_steps=None, clearance=None, verbose=False):
    from yam_arena import ik
    from yam_arena.constants import READY

    from rdb_tablets_cap import fixed_scene as fixture
    from rdb_tablets_cap import layout as L
    from rdb_tablets_cap.grading import TabletGrade
    from rdb_tablets_cap.scripted import POUR_DIR, TabletReference

    reference = TabletReference()
    actions = reference.sequence()
    grade = TabletGrade()
    dt = 1 / fixture.CONTROL_HZ
    blend = 1 - np.exp(-dt / arm_lag_s)
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    bias = np.asarray(joint_bias, dtype=float)
    pose = {name: [np.asarray(fixture.START[name], dtype=float), np.eye(3)] for name in ("bottle", "cap", "cup")}
    width = {"bottle": 2 * L.BOTTLE_R, "cap": 2 * L.CAP_R}
    tall = {"bottle": .07, "cap": L.CAP_H}
    held = {}                                # arm -> (name, relative rotation, relative point)
    where = ["bottle"] * L.N_TABLETS         # bottle, neck, cap, cup, table
    spot = [np.zeros(3) for _ in range(L.N_TABLETS)]
    neck_until = {}
    bursts = list(bursts)
    next_out, next_tip = 0., 0.
    report = {"worst_limit_margin_deg": 180., "min_arm_gap_mm": np.inf, "min_table_gap_mm": np.inf, "grasps": []}
    result = None
    steps = max_steps or fixture.MAX_STEPS

    def tablet_points():
        points = np.zeros((L.N_TABLETS, 3))
        slots = {"cap": 0, "cup": 0}
        for i, place in enumerate(where):
            if place in ("bottle", "neck"):
                local = np.asarray(L.tablet_start(i)) if place == "bottle" else np.array((0., 0., L.SHOULDER_H + .006))
                points[i] = pose["bottle"][0] + pose["bottle"][1] @ local
            elif place == "cap":
                k = slots["cap"]; slots["cap"] += 1
                local = np.array((.008 * np.cos(2.4 * k), .008 * np.sin(2.4 * k), L.CAP_FLOOR + .0025 + .001 * k))
                points[i] = pose["cap"][0] + pose["cap"][1] @ local
            elif place == "cup":
                k = slots["cup"]; slots["cup"] += 1
                points[i] = pose["cup"][0] + pose["cup"][1] @ np.array((.004 * k, 0., .004 + .003 * k))
            else:
                points[i] = spot[i]
        return points

    def land(i, point):
        cap_local = pose["cap"][1].T @ (point - pose["cap"][0])
        cup_local = pose["cup"][1].T @ (point - pose["cup"][0])
        mouth = pose["bottle"][0] + pose["bottle"][1] @ (0., 0., L.BOTTLE_H)
        if np.hypot(*(point - mouth)[:2]) < L.NECK_R_IN - .003 and point[2] > mouth[2] and pose["bottle"][1][2, 2] > .95:
            where[i] = "bottle"
        elif np.hypot(*cap_local[:2]) < L.CAP_R_IN - .003 and cap_local[2] > 0 and pose["cap"][1][2, 2] > .95:
            where[i] = "cap"
        elif np.hypot(*cup_local[:2]) < L.cup_inner_radius(L.CUP_H) - .004 and cup_local[2] > 0:
            where[i] = "cup"
        else:
            where[i], spot[i] = "table", np.array((point[0], point[1], L.TABLET_T / 2))

    for tick in range(1, steps + 1):
        now = tick * dt
        state = {name: (pose[name][0].copy(), pose[name][1].copy()) for name in pose}
        state["tablets"] = tablet_points()
        reference.observe(joints, state)
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        target = command.copy()
        target[:6] += bias
        target[7:13] += bias * (1, -1, -1, 1, 1, 1) * 0 + bias
        joints = joints + blend * (target - joints)
        tcps, clouds = [], []
        for i, arm in enumerate(fixture.ARMS):
            point, rotation = ik.tcp_pose(joints[7 * i:7 * i + 6], fixture.BASES[arm])
            tcps.append(point)
            gap = joints[7 * i + 6] * .0939
            if arm in held:
                name, relative_rotation, relative_point = held[arm]
                if gap > width[name] + .006:
                    del held[arm]                                 # let go: it stays standing where it is
                    origin = pose[name][0]
                    pose[name] = [np.array((origin[0], origin[1], 0.)), np.eye(3)]
                else:
                    object_rotation = rotation @ relative_rotation.T
                    pose[name] = [point - object_rotation @ relative_point, object_rotation]
            else:
                for name in ("bottle", "cap"):
                    if any(h[0] == name for h in held.values()):
                        continue
                    local = pose[name][1].T @ (point - pose[name][0])
                    if gap < width[name] + .002 and np.hypot(local[0], local[1]) < .010 and 0 < local[2] < tall[name]:
                        held[arm] = (name, pose[name][1].T @ rotation, pose[name][1].T @ (point - pose[name][0]))
                        report["grasps"].append((tick, arm, name, [round(1000 * float(v), 1) for v in local]))
            if clearance is not None and tick % 3 == 0:
                cloud = clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fixture.BASES[arm],
                                            links=("gripper", "tip_left", "tip_right", "link5", "link4", "link3"))
                clouds.append(cloud)
                report["min_table_gap_mm"] = min(report["min_table_gap_mm"], 1000 * float(cloud[:, 2].min()))
                for name, radius, height in (("bottle", L.BOTTLE_R, L.BOTTLE_H), ("cap", L.CAP_R, L.CAP_H), ("cup", L.CUP_R_TOP, L.CUP_H)):
                    if held.get(arm, (None,))[0] == name:
                        continue                                  # the gripper holding it touches it on purpose
                    local = (cloud - pose[name][0]) @ pose[name][1]
                    radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - radius, 0.)
                    axial = np.maximum(np.maximum(-local[:, 2], local[:, 2] - height), 0.)
                    gap = 1000 * float(np.hypot(radial, axial).min())
                    key = f"min_{arm}_arm_to_{name}_mm"
                    if gap < report.get(key, (np.inf,))[0]:
                        report[key] = (round(gap, 1), tick, reference.phases[-1][1] if reference.phases else "")

        if "left" in held and held["left"][0] == "cap" and not any(h[0] == "bottle" for h in held.values()):
            rim = pose["cap"][0][:, None] + pose["cap"][1] @ np.array([[L.CAP_R * np.cos(a), L.CAP_R * np.sin(a), z]
                                                           for a in np.linspace(0, 2 * np.pi, 24, endpoint=False)
                                                           for z in (0., L.CAP_H)]).T
            local = (rim.T - pose["bottle"][0]) @ pose["bottle"][1]
            radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - L.BOTTLE_R * (local[:, 2] < L.BODY_H) - (L.NECK_R_OUT + .0012) * (local[:, 2] >= L.BODY_H), 0.)
            axial = np.maximum(np.maximum(-local[:, 2], local[:, 2] - L.BOTTLE_H), 0.)
            gap = 1000 * float(np.hypot(radial, axial).min())
            if gap < report.get("min_cap_to_bottle_mm", (np.inf,))[0]:
                report["min_cap_to_bottle_mm"] = (round(gap, 1), tick, reference.phases[-1][1] if reference.phases else "")
        if len(clouds) == 2:
            report["min_arm_gap_mm"] = min(report["min_arm_gap_mm"], 1000 * clearance.arm_gap(*clouds))
        # -- the toy tablet rules
        bottle_rotation = pose["bottle"][1]
        tilt = np.degrees(np.arccos(np.clip(bottle_rotation[2, 2], -1., 1.)))
        for i, until in list(neck_until.items()):
            if now >= until:
                del neck_until[i]
                lip = pose["bottle"][0] + bottle_rotation @ (L.NECK_R_IN * POUR_DIR[0], L.NECK_R_IN * POUR_DIR[1], L.BOTTLE_H)
                land(i, lip + POUR_DIR * (.008 + miss_mm / 1000))
            elif tilt < spill_deg - 25:
                del neck_until[i]
                where[i] = "bottle"
        if tilt > spill_deg and now >= next_out and "bottle" in where:
            count = bursts.pop(0) if bursts else 1
            for _ in range(count):
                if "bottle" in where:
                    i = where.index("bottle")
                    where[i] = "neck"
                    neck_until[i] = now + .3
            next_out = now + .6
        cap_rotation = pose["cap"][1]
        if np.degrees(np.arccos(np.clip(cap_rotation[2, 2], -1., 1.))) > cap_spill_deg and now >= next_tip and "cap" in where:
            low = np.array((cap_rotation[2, 0], cap_rotation[2, 1], 0.))
            low = -low / np.linalg.norm(low)                      # local direction of the low side of the rim
            rim = pose["cap"][0] + cap_rotation @ (L.CAP_R_IN * low + (0., 0., L.CAP_H))
            for _ in range(cap_burst):
                if "cap" in where:
                    land(where.index("cap"), rim + .003 * (cap_rotation @ low) * (1, 1, 0))
            next_tip = now + cap_gap_s
        result = grade.update(tablet_points(), tuple(pose["bottle"]), tuple(pose["cap"]), tuple(pose["cup"]),
                              np.zeros(L.N_TABLETS, dtype=bool), tcps,
                              cap_held=any(h[0] == "cap" for h in held.values()), cap_supported=False)
        if verbose and tick % 40 == 0:
            print(tick, reference.phases[-1][1] if reference.phases else "", round(tilt, 1), result["outcome"], result["score"],
                  result["in_bottle"], result["in_cap"], result["in_cup"], result["strays"])
        if result["score"] == 1. or grade.finished:
            break
    report.update(score=result["score"], outcome=result["outcome"], void=result["void"], steps=tick,
                  in_cup=result["in_cup"], in_bottle=result["in_bottle"], strays=result["strays"],
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), pours=reference.log["pours"],
                  events=reference.log["events"], reference_grasps=reference.log["grasps"],
                  phases=[name for _, name in reference.phases])
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
    for name, kw in (("one at a time", {}),
                     ("a rush of four first, then singles", dict(bursts=(4, 1, 1, 1, 1))),
                     ("one, then a pause, slow sagging arm", dict(bursts=(1, 0, 0, 1, 1), arm_lag_s=.2,
                                                                 joint_bias=(0., .02, -.015, 0., 0., 0.))),
                     ("three in the cap, two slide back together (re-pour)", dict(bursts=(3, 1, 1, 1), cap_burst=2)),
                     ("every tablet lands 30 mm past the cap", dict(miss_mm=30.))):
        report = dry_run(clearance=clearance if name.startswith(("one at", "a rush", "three")) else None, verbose=args.verbose, **kw)
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
