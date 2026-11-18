"""Dry run of the scripted demonstration without the simulator, scored by the real scorer.

The arms follow their joint commands with a first-order lag (no contact physics). The sprayer stands still on
its mark (optionally turned a few degrees). The valve's cap follows a gripper that has closed on it, out along
the valve's axis between its stops, and springs back when let go. The spray head follows a gripper that has
closed on the nozzle, round the sprayer's axis, forward only and up to the thread's end, but only while the
other hand holds the pump knob: unheld, the push on the nozzle tips the sprayer over instead (it takes 2 N at
that height; turning the head takes 5 N). A hand that closes on the pump knob stays where it closed
(the knob does not give). The pressure is the
same rule the simulator uses (``layout.pressure_step``). The demonstration gets the same state it gets in the
simulator and ``grading.py`` scores every tick. This checks reach, joint limits, clearances between the arms
and from the arms to the sprayer, the control flow and the retries; it says nothing about finger contact,
friction or whether the sprayer stays put.

Run:  python tools/dryrun_demo.py [--yam-arena <checkout>] [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sprayer_cloud(head_angle, yaw, origin):
    """Points on the sprayer's surface in the tabletop frame: (never-touch parts, parts the demo handles)."""
    from rdb_sprayer_relief import layout as L

    def rings(r0, z0, r1, z1, n=10, m=40):
        t = np.linspace(0., 1., n)[:, None]
        a = np.linspace(0., 2 * np.pi, m, endpoint=False)[None, :]
        r, z = r0 + (r1 - r0) * t, z0 + (z1 - z0) * t
        return np.stack((r * np.cos(a), r * np.sin(a), z + 0 * a), axis=-1).reshape(-1, 3)

    def grid(azimuth, outline, width, n=8):
        out = np.array((np.cos(azimuth), np.sin(azimuth), 0.))
        across = np.array((-np.sin(azimuth), np.cos(azimuth), 0.))
        (r0, z0), (r1, z1), (r2, z2), (r3, z3) = outline
        pts = []
        for u in np.linspace(0., 1., n):
            for v in np.linspace(0., 1., n):
                r = (1 - u) * ((1 - v) * r0 + v * r1) + u * ((1 - v) * r3 + v * r2)
                z = (1 - u) * ((1 - v) * z0 + v * z1) + u * ((1 - v) * z3 + v * z2)
                for s in (-1., 0., 1.):
                    pts.append(out * r + across * s * width / 2 + (0., 0., z))
        return np.asarray(pts)

    nozzle = L.nozzle_azimuth(head_angle)
    fixed = [rings(L.TANK_R, 0., L.TANK_R, L.BODY_TOP), rings(L.TANK_R, L.BODY_TOP, L.NECK_R, L.NECK_Z),
             rings(L.NECK_R, L.NECK_Z, L.COLLAR_TOP_R, L.COLLAR_TOP), rings(L.HEAD_R, L.COLLAR_TOP, L.HEAD_R, L.HEAD_TOP, 4),
             rings(L.STEM_R, L.HEAD_TOP, L.STEM_R, L.KNOB_BOTTOM, 5, 12),
             grid(nozzle + np.pi, [(.046, .270), (.064, .270), (.084, L.HANDLE_BOTTOM), (.066, L.HANDLE_BOTTOM)], L.HANDLE_W),
             grid(nozzle + np.pi, [(.028, .256), (.062, .256), (.062, .282), (.028, .282)], L.HANDLE_W)]
    handled = [rings(L.KNOB_R, L.KNOB_BOTTOM, L.KNOB_R, L.HEIGHT, 4),
               grid(nozzle, [(.028, L.NOZZLE_Z - .006), (L.NOZZLE_TIP_R, L.NOZZLE_Z - .006), (L.NOZZLE_TIP_R, L.NOZZLE_Z + .006), (.028, L.NOZZLE_Z + .006)], L.NOZZLE_W, 6),
               L.valve_cap_centre()[None, :] + np.zeros((1, 3))]
    turn = np.array(((np.cos(yaw), -np.sin(yaw), 0.), (np.sin(yaw), np.cos(yaw), 0.), (0., 0., 1.)))
    return tuple(np.concatenate(group) @ turn.T + origin for group in (fixed, handled))


def dry_run(*, variant="full", arm_lag_s=.12, tank_yaw_deg=0., miss_first_pinch=False, valve_slips_once=False, start_bar=None,
            sag=(0., 0., 0.), max_steps=None, clearance=None, verbose=False, stop_after=None):
    from scipy.spatial import cKDTree
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, READY

    from rdb_sprayer_relief import fixed_scene as fixture
    from rdb_sprayer_relief import layout as L
    from rdb_sprayer_relief.grading import SprayerGrade
    from rdb_sprayer_relief.scripted import KNOB_Z, SprayerReference

    reference = SprayerReference(variant=variant)
    actions = reference.sequence()
    grade = SprayerGrade()
    dt = 1 / fixture.CONTROL_HZ
    blend = 1 - np.exp(-dt / arm_lag_s)
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    yaw = np.radians(tank_yaw_deg)
    tank_rot = np.array(((np.cos(yaw), -np.sin(yaw), 0.), (np.sin(yaw), np.cos(yaw), 0.), (0., 0., 1.)))
    origin = fixture.ORIGIN.copy()
    bar = fixture.START_BAR if start_bar is None else float(start_bar)
    lift, head = 0., 0.
    valve_grip = nozzle_grip = None          # (arm index, reference value) while a gripper holds the part
    misses = 1 if miss_first_pinch else 0
    slips = 1 if valve_slips_once else 0
    report = {"worst_limit_margin_deg": 180., "min_arm_gap_mm": np.inf, "min_arm_to_sprayer_mm": np.inf,
              "min_tips_to_untouched_parts_mm": np.inf, "events": [], "tank_tipped_deg": 0.}
    result = None
    steps = max_steps or fixture.MAX_STEPS
    out = tank_rot @ L.valve_axis()
    tilt = 0.
    braced = None                            # (arm index, where its tool point stopped) while a hand holds the knob
    gaps = {}
    sag = np.asarray(sag, dtype=float)
    for tick in range(1, steps + 1):
        state = {"bar": bar, "valve": lift, "head": head, "tank_pos": origin.copy(), "tank_rot": tank_rot.copy()}
        measured = joints.copy()
        for i in range(2):                    # a gripper closed on a part reads the part's width, not its command
            width = measured[7 * i + 6] * 2 * abs(float(FINGER_OPEN))
            if valve_grip is not None and valve_grip[0] == i and valve_grip[1] is not None:
                measured[7 * i + 6] = max(width, L.VALVE_CAP) / (2 * abs(float(FINGER_OPEN)))
            if nozzle_grip is not None and nozzle_grip[0] == i and nozzle_grip[1] is not None:
                measured[7 * i + 6] = max(width, L.NOZZLE_W) / (2 * abs(float(FINGER_OPEN)))
            if braced is not None and braced[0] == i:
                measured[7 * i + 6] = max(width, 2 * L.KNOB_R) / (2 * abs(float(FINGER_OPEN)))
        reference.observe(measured, state)
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fixture.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        joints = joints + blend * (command - joints)
        tools = [ik.tcp_pose(joints[7 * i:7 * i + 6], fixture.BASES[arm]) for i, arm in enumerate(fixture.ARMS)]
        knob = origin + (0., 0., KNOB_Z)
        for i, arm in enumerate(fixture.ARMS):       # a hand closed on the pump knob stays where it closed
            point, rotation = tools[i]
            width = joints[7 * i + 6] * 2 * abs(float(FINGER_OPEN))
            if braced is None and width <= 2 * L.KNOB_R + .001 and np.linalg.norm(point + rotation[:, 2] * .0065 - knob) < .008:
                braced = (i, point.copy())
                report["events"].append((tick, f"{arm} pads on the pump knob"))
            elif braced is not None and braced[0] == i:
                if width > 2 * L.KNOB_R + .004:
                    braced = None
                    report["events"].append((tick, f"{arm} let go of the pump knob"))
                else:
                    q, _, _ = ik.solve(braced[1], ik.matrix_to_quat_xyzw(rotation), joints[7 * i:7 * i + 6], fixture.BASES[arm], iters=40)
                    joints[7 * i:7 * i + 6] = q
                    tools[i] = ik.tcp_pose(q, fixture.BASES[arm])

        nozzle_az = yaw + L.nozzle_azimuth(head)
        centre = origin + (0., 0., L.NOZZLE_Z)
        for i, (point, rotation) in enumerate(tools):
            point = point + sag
            width = joints[7 * i + 6] * 2 * abs(float(FINGER_OPEN))
            # the valve's cap
            if valve_grip is None and width <= L.VALVE_CAP + .001:
                rel = point - (origin + tank_rot @ L.valve_base())
                along = float(rel @ out)
                off = float(np.linalg.norm(rel - out * along))
                square = abs(rotation[:, 2] @ -out) > np.cos(np.radians(12.))
                if off < .005 and .008 < along < .028 and square:
                    if slips:
                        slips -= 1
                        report["events"].append((tick, "valve cap slipped out of the pads"))
                        valve_grip = (i, None)
                    else:
                        valve_grip = (i, along - lift)
                        report["events"].append((tick, f"{fixture.ARMS[i]} pads on the valve cap"))
            elif valve_grip is not None and valve_grip[0] == i:
                if width > L.VALVE_CAP + .004:
                    valve_grip, lift = None, 0.
                    report["events"].append((tick, "valve let go"))
                elif valve_grip[1] is not None:
                    rel = point - (origin + tank_rot @ L.valve_base())
                    lift = float(np.clip(rel @ out - valve_grip[1], 0., L.VALVE_TRAVEL))
            # the nozzle
            pads = point + rotation[:, 2] * .0065       # the middle of the pads
            if nozzle_grip is None and width <= L.NOZZLE_W + .001 and (valve_grip is None or valve_grip[0] != i):
                rel = pads - centre
                radius = float(np.hypot(rel[0], rel[1]))
                az = float(np.arctan2(rel[1], rel[0]))
                off = (az - nozzle_az + np.pi) % (2 * np.pi) - np.pi
                upright = abs(rotation[2, 0]) > np.cos(np.radians(12.))          # one pad above the other
                if .035 < radius < L.NOZZLE_TIP_R - .002 and abs(off) * radius < .006 and abs(rel[2]) < .012 and upright:
                    if misses:
                        misses -= 1
                        report["events"].append((tick, "nozzle pinch missed"))
                        nozzle_grip = (i, None)
                    else:
                        nozzle_grip = (i, off)
                        report["events"].append((tick, f"{fixture.ARMS[i]} pads on the nozzle at {np.degrees(nozzle_az) % 360:.0f} deg"))
            elif nozzle_grip is not None and nozzle_grip[0] == i:
                if width > L.NOZZLE_W + .004:
                    nozzle_grip = None
                    report["events"].append((tick, f"nozzle let go, head {np.degrees(head):.0f} deg"))
                elif nozzle_grip[1] is not None:
                    rel = pads - centre
                    az = float(np.arctan2(rel[1], rel[0]))
                    ahead = (az - nozzle_grip[1] - nozzle_az + np.pi) % (2 * np.pi) - np.pi
                    drag = np.radians(2.)                         # the head trails the hand by the arm's give
                    if ahead > drag:
                        if braced is not None and braced[0] != i:
                            head = float(min(head + ahead - drag, L.HEAD_FREE))
                        else:                             # unheld: the push tips the sprayer away from the hand's motion
                            tilt += ahead - drag
                            tangent = np.array((-np.sin(nozzle_az), np.cos(nozzle_az), 0.))
                            axis = np.cross((0., 0., 1.), tangent)
                            from scipy.spatial.transform import Rotation as _R
                            tank_rot = _R.from_rotvec(axis * tilt).as_matrix() @ np.array(
                                ((np.cos(yaw), -np.sin(yaw), 0.), (np.sin(yaw), np.cos(yaw), 0.), (0., 0., 1.)))
                            out = tank_rot @ L.valve_axis()
                            report["tank_tipped_deg"] = round(float(np.degrees(tilt)), 1)
                        nozzle_az = yaw + L.nozzle_azimuth(head)
        bar = L.pressure_step(bar, lift, head, dt)

        if clearance is not None and tick % 4 == 0:
            fixed, handled = sprayer_cloud(head, yaw, origin)
            everything = cKDTree(np.concatenate((fixed, handled)))
            untouched = cKDTree(fixed)
            phase = reference.phases[-1][1] if reference.phases else ""
            row = gaps.setdefault(phase, {"arms": np.inf, "idle_or_working_arm_to_sprayer": np.inf, "bracing_arm_to_other_parts": np.inf,
                                          "tips_to_untouched_parts": np.inf})
            clouds = []
            for i, arm in enumerate(fixture.ARMS):
                q, jaw = joints[7 * i:7 * i + 6], joints[7 * i + 6]
                arm_links = clearance.arm_cloud(q, jaw, fixture.BASES[arm], links=("link1", "link2", "link3", "link4", "link5", "gripper"))
                tips = clearance.arm_cloud(q, jaw, fixture.BASES[arm], links=("tip_left", "tip_right"))
                if reference.bracing == arm or phase.startswith(("hold the knob", "let go of the knob")) and arm in phase:
                    row["bracing_arm_to_other_parts"] = min(row["bracing_arm_to_other_parts"], 1000 * float(untouched.query(np.concatenate((arm_links, tips)))[0].min()))
                else:
                    row["idle_or_working_arm_to_sprayer"] = min(row["idle_or_working_arm_to_sprayer"], 1000 * float(everything.query(arm_links)[0].min()))
                    row["tips_to_untouched_parts"] = min(row["tips_to_untouched_parts"], 1000 * float(untouched.query(tips)[0].min()))
                clouds.append(np.concatenate((arm_links, tips)))
            row["arms"] = min(row["arms"], 1000 * clearance.arm_gap(*clouds))
            report["min_arm_gap_mm"] = min(report["min_arm_gap_mm"], row["arms"])
            report["min_arm_to_sprayer_mm"] = min(report["min_arm_to_sprayer_mm"], row["idle_or_working_arm_to_sprayer"], row["bracing_arm_to_other_parts"])
            report["min_tips_to_untouched_parts_mm"] = min(report["min_tips_to_untouched_parts_mm"], row["tips_to_untouched_parts"])

        result = grade.update(bar, lift, head, origin - fixture.ORIGIN, tank_rot[:, 2], [point - fixture.ORIGIN for point, _ in tools])
        if verbose and tick % 20 == 0:
            print(tick, result["score"], round(bar, 2), round(1000 * lift, 1), round(np.degrees(head)),
                  [np.round(1000 * p).astype(int).tolist() for p, _ in tools], reference.phases[-1][1] if reference.phases else "")
        if result["score"] == 1. or grade.finished or (stop_after and reference.phases and reference.phases[-1][1].startswith(stop_after)):
            break
        if reference.done and tick > reference.phases[-1][0] + 3 * fixture.CONTROL_HZ:
            break
    report.update(gaps_by_phase_mm={k: {n: round(v, 1) for n, v in row.items()} for k, row in gaps.items()}, score=result["score"], steps=tick, grade={k: result[k] for k in ("bar", "vented_first", "hazard", "head_deg", "standing", "grippers_clear")},
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), phases=reference.phases, log=reference.log)
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
             ("sprayer turned 8 degrees on its mat, slow arm, first nozzle pinch misses, valve cap slips once",
              dict(tank_yaw_deg=8., arm_lag_s=.2, miss_first_pinch=True, valve_slips_once=True)),
             ("control: valve skipped (must score 0)", dict(variant="no_vent")),
             ("control: pressure let out, nothing else (must score 0.3)", dict(variant="vent_only")),
             ("control: sprayer already empty of pressure", dict(start_bar=0.)),
             ("control: the sprayer is not held (the push tips it; must score 0)", dict(variant="no_hold")))
    for name, kw in cases:
        report = dry_run(clearance=clearance if name == "nominal" else None, verbose=args.verbose, **kw)
        log = report.pop("log")
        phases = report.pop("phases")
        gaps = report.pop("gaps_by_phase_mm")
        for phase, row in gaps.items():
            print("   ", phase, row)
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()})
        if args.verbose:
            print("   phases:", phases)
            for event in log["events"]:
                print("   ", event)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
