"""Dry run of the scripted demonstration without the simulator, scored by the real air rule and scorer.

The arms follow their joint commands with a first-order lag (no contact physics, no sag). The bellows board
reopens at up to 0.8 rad/s to its open stop unless the left hand's lower finger is in the way: the board's outer
top edge cannot pass through the plane of that finger's outer face where the finger reaches past the edge, so the
board is pushed shut exactly as far as the finger goes. A key goes down as far as the right hand's lower pad tip
(three points along its 12 mm end edge) is below the key's top, up to its 7 mm stop. ``air.py`` turns board travel
and key depths into reservoir air and sounding reeds, and ``grading.py`` scores every tick. The demonstration gets
the state it gets in the simulator. This checks reach, joint limits, the control flow, the timing and the
two-hand logic; it says nothing about forces or friction.

``wrist_lag_s`` gives joints 4 to 6 their own, longer lag: with shoulder 0.07 s and wrist 0.15 s this model
matches the key hand's measured path in Isaac runs 3 and 4 to 5 to 7 mm RMS, and it reproduced run 4's failure
(0.75, F4 sounding 0.45 s, a 1.7 s overlap) when run on run 4's demonstration. ``key_hand_y`` and ``key_hand_z`` shift the key hand's working point along the keyboard and up, uncorrected (the
depth servo must make up a height error; in runs 4 and 5 the pressing hand rode 6 to 7 mm above its plan). ``load_give`` shifts the left hand's working point by (x, z) metres whenever it is pushing the board, a crude
stand-in for the compliant arm yielding under the board's 12 N (the demonstration's bias and sink must make up
for it). Controls (``--variant``): ``nominal``; ``no_pump`` (the left hand never pushes: the reservoir stays empty);
``pump_only`` (the right hand never presses a key); ``prime_then_stop`` (the left hand fills the reservoir once,
then stops; the right hand plays as usual). The last three must score 0.

Run:  python tools/dryrun_demo.py [--clearance <folder with clearance.py>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def tip_points(rotation, tcp):
    """The end edge of the right hand's lower pad tip: five points across the pad's 15 mm."""
    from rdb_harmonium.scripted import FACE, KEY_TIP

    x, y, a = rotation[:, 0], rotation[:, 1], rotation[:, 2]
    edge = tcp + KEY_TIP * a - FACE * x
    return [edge + s * y for s in (-.0075, -.0035, 0., .0035, .0075)]


def key_under(point):
    """(key name, its top at rest) of the key whose top the point is over, black keys first, or (None, None)."""
    from rdb_harmonium import fixed_scene as fx
    from rdb_harmonium import layout as L

    local = np.asarray(point) - fx.CASE_START
    best = (None, None)
    for midi, name in zip(L.KEY_MIDI, L.KEY_NAMES, strict=True):
        x, y, top, sx, sy, _, _, white = L.key_box(midi)
        if abs(local[0] - x) <= sx / 2 and abs(local[1] - y) <= sy / 2:
            if not white:
                return name, top
            best = (name, top)
    return best


def finger_on_flap(rotation, tcp, theta, y):
    """Signed distance (m) of the board's outer top edge from the left hand's lower-finger face (negative: inside
    the finger), and whether the finger reaches past the edge there."""
    from rdb_harmonium import fixed_scene as fx
    from rdb_harmonium.scripted import PUMP_FACE

    x, yv, a = rotation[:, 0], rotation[:, 1], rotation[:, 2]
    corner = fx.flap_corner(theta, y)
    face = tcp - PUMP_FACE * x
    along = float((corner - (tcp + .013 * a)) @ -a)        # from the pad tip end back up the finger
    across = float((corner - tcp) @ yv)
    return float((corner - face) @ -x), (-.002 <= along <= .060 and abs(across) <= .026)


def dry_run(*, variant="nominal", arm_lag_s=.12, wrist_lag_s=None, load_give=(0., 0.), key_hand_y=0., key_hand_z=0., clearance=None, max_steps=None, verbose=False):
    from yam_arena.constants import READY

    from rdb_harmonium import fixed_scene as fx
    from rdb_harmonium import layout as L
    from rdb_harmonium.air import Air
    from rdb_harmonium.grading import PhraseGrade
    from rdb_harmonium.scripted import CHAIN, HarmoniumReference

    reference = HarmoniumReference()
    actions = reference.sequence()
    grade = PhraseGrade(start_xy=fx.CASE_START[:2])
    air = Air()
    dt = 1 / fx.CONTROL_HZ
    blend = np.full(14, 1 - np.exp(-dt / arm_lag_s))
    if wrist_lag_s is not None:                     # joints 4 to 6 (soft wrist drives) follow more slowly
        blend[[3, 4, 5, 10, 11, 12]] = 1 - np.exp(-dt / wrist_lag_s)
    joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
    theta = L.FLAP_OPEN
    depths = dict.fromkeys(L.KEY_NAMES, 0.)
    report = {"worst_limit_margin_deg": 180., "min_arm_gap_mm": np.inf, "min_hull_to_case_mm": np.inf, "min_tip_to_unmarked_keys_mm": np.inf,
              "max_volume_l": 0., "min_volume_while_playing_l": np.inf, "min_flap_rad": np.inf}
    result, primed, tick, left_on = None, False, 0, False
    frames, stroke_ticks, right_tcp = [], [], []
    steps = max_steps or fx.MAX_STEPS
    from yam_arena import ik
    for tick in range(1, steps + 1):
        state = {"flap": theta, "volume": air.volume, "depths": dict(depths), "sounding": air.sounding}
        reference.observe(joints.copy(), state)
        command = np.asarray(next(actions).actions[0].data, dtype=float)
        for i in range(2):
            q = command[7 * i:7 * i + 6]
            if np.any(q < ik.LOWER - 1e-6) or np.any(q > ik.UPPER + 1e-6):
                raise RuntimeError(f"{fx.ARMS[i]} arm commanded past a joint limit at tick {tick}: {q}")
            report["worst_limit_margin_deg"] = min(report["worst_limit_margin_deg"],
                                                   float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q).min())))
        if variant == "no_pump" or (variant == "prime_then_stop" and primed and air.volume < L.RESERVOIR_M3):
            command[0:6] = joints[0:6]                    # the left arm stays where it is
        if variant == "pump_only":
            command[7:13] = joints[7:13]
        joints = joints + blend * (command - joints)
        poses = {arm: CHAIN.fk(joints[7 * i:7 * i + 6], fx.BASES[arm])[:2] for i, arm in enumerate(fx.ARMS)}
        # the bellows board: reopens unless the left finger is in the way
        tcp, rot = poses["left"]
        if left_on:                                 # the compliant arm gives way under the board's 12 N
            tcp = tcp + (load_give[0], 0., load_give[1])
        free = min(L.FLAP_OPEN, theta + L.FLAP_OPEN_SPEED * dt)
        new = free
        gap, over = finger_on_flap(rot, tcp, free, fx.PUMP_Y)
        if over and gap < 0.:                       # the finger is in the way: the edge rests on its face
            lo, hi = 0., free
            for _ in range(40):
                mid = (lo + hi) / 2
                g, o = finger_on_flap(rot, tcp, mid, fx.PUMP_Y)
                lo, hi = (mid, hi) if (not o or g >= 0.) else (lo, mid)
            new = lo
        gap, over = finger_on_flap(rot, tcp, new, fx.PUMP_Y)
        left_on = over and gap < .002
        theta = max(new, 0.)
        report["min_flap_rad"] = min(report["min_flap_rad"], theta)
        # the keys
        depths = dict.fromkeys(L.KEY_NAMES, 0.)
        right_on = False
        tcp, rot = poses["right"]
        tcp = tcp + (0., key_hand_y, key_hand_z)    # uncorrected errors of the key hand: along the keyboard, height
        for point in tip_points(rot, tcp):
            name, top = key_under(point)
            if name is None:
                continue
            down = (fx.CASE_START[2] + top) - point[2]
            if down > -.002:
                right_on = True
            depths[name] = max(depths[name], float(np.clip(down, 0., L.KEY_TRAVEL)))
        before = air.strokes
        p, sounding = air.step(dt, theta, depths)
        frames.append((sounding, p))
        right_tcp.append(poses["right"][0].copy())
        if air.strokes > before:
            stroke_ticks.append(tick)
        primed = primed or air.volume >= .9 * L.RESERVOIR_M3
        report["max_volume_l"] = max(report["max_volume_l"], 1000 * air.volume)
        if reference.phases and reference.phases[-1][1].startswith("note"):
            report["min_volume_while_playing_l"] = min(report["min_volume_while_playing_l"], 1000 * air.volume)
        keys_up = max(depths.values()) < .001
        result = grade.update(sounding, fx.CASE_START, np.eye(3), left_on or right_on, keys_up,
                              fed=air.fed_m3 > 0.)
        if clearance is not None and tick % 4 == 0:
            clouds = [clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fx.BASES[arm])
                      for i, arm in enumerate(fx.ARMS)]
            report["min_arm_gap_mm"] = min(report["min_arm_gap_mm"], 1000 * clearance.arm_gap(*clouds))
            if any(depths[n] > 0. for n in L.PHRASE):
                tips = clearance.arm_cloud(joints[7:13], joints[13], fx.BASES["right"], links=("tip_left", "tip_right"))
                for midi, name in zip(L.KEY_MIDI, L.KEY_NAMES, strict=True):
                    if name in L.PHRASE:          # keys outside the phrase: the black keys beside them, B3, F4
                        continue
                    x, y, top, sx, sy, t, _, _ = L.key_box(midi)
                    centre = fx.to_station((x, y, top - t / 2))
                    report["min_tip_to_unmarked_keys_mm"] = min(report["min_tip_to_unmarked_keys_mm"], 1000 * clearance.box_gap(
                        tips, centre, (sx / 2, sy / 2, t / 2)))
            for i, arm in enumerate(fx.ARMS):
                body = clearance.arm_cloud(joints[7 * i:7 * i + 6], joints[7 * i + 6], fx.BASES[arm],
                                           links=("link3", "link4", "link5", "gripper"))
                for centre, half in case_boxes():
                    report["min_hull_to_case_mm"] = min(report["min_hull_to_case_mm"],
                                                        1000 * clearance.box_gap(body, centre, half))
        if verbose and tick % 20 == 0:
            print(tick, reference.phases[-1][1], round(theta, 3), round(1000 * air.volume, 2), sounding,
                  result["longest_s"], result["score"])
        if grade.finished:
            break
    report.update(score=result["score"], played=result["played"], steps=tick, longest_s=result["longest_s"],
                  joins_s=result["joins_s"], other_keys_s=result["other_keys_s"], released=result["released"],
                  strokes=air.strokes, fed_l=round(1000 * air.fed_m3, 2), vented_l=round(1000 * air.vented_m3, 2),
                  worst_ik_mm=round(1000 * reference.worst_ik, 3), phases=reference.phases,
                  full_reservoir_one_reed_s=round(Air(volume=L.RESERVOIR_M3).seconds_left(1), 2), frames=frames, stroke_ticks=stroke_ticks, right_tcp=right_tcp)
    return report


def case_boxes():
    """Axis-aligned boxes (centre, half sizes) of the case, tabletop frame, for the hull clearance check: the case
    under the keys, the stop rail and the top board, and the cheeks beside the keyboard (the keys and the bellows
    board are the hands' work)."""
    from rdb_harmonium import fixed_scene as fx
    from rdb_harmonium import layout as L

    x0 = fx.CASE_START[0]
    boxes = [((x0 + L.BACK_X / 2, 0., L.KEYBED_Z / 2), (L.BACK_X / 2, L.CASE_W / 2, L.KEYBED_Z / 2)),
             ((x0 + (L.STOP_RAIL[0] + L.BACK_X) / 2, 0., L.CASE_H / 2),
              ((L.BACK_X - L.STOP_RAIL[0]) / 2, L.CASE_W / 2, L.CASE_H / 2))]
    for side in (1, -1):                            # the cheeks beside the keyboard
        y0 = side * (L.KEYBOARD_LEFT_Y + L.CASE_W / 2) / 2
        boxes.append(((x0 + L.STOP_RAIL[0] / 2, y0, (L.KEYBED_Z + L.CASE_H - .012) / 2),
                      (L.STOP_RAIL[0] / 2, (L.CASE_W / 2 - L.KEYBOARD_LEFT_Y) / 2, (L.CASE_H - .012 - L.KEYBED_Z) / 2)))
    return boxes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clearance", help="folder holding clearance.py (arm collision hulls), optional")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    for path in (ROOT, ROOT / "tools", ROOT / "tools/stubs"):
        sys.path.insert(0, str(path))
    clearance = None
    if args.clearance:
        sys.path.insert(0, args.clearance)
        import clearance
    for name, kw in (("nominal", dict(clearance=clearance)), ("slow arms", dict(arm_lag_s=.25)),
                     ("pushing hand gives 30 mm back and 8 mm up under load", dict(load_give=(.030, .008))),
                     ("fitted to Isaac runs 3 and 4: shoulder joints lag 0.07 s, wrist joints 0.15 s",
                      dict(arm_lag_s=.07, wrist_lag_s=.15)),
                     ("wrist lagging more: 0.05 s and 0.25 s", dict(arm_lag_s=.05, wrist_lag_s=.25)),
                     ("fitted lags, key hand riding 7 mm high (runs 4 and 5)", dict(arm_lag_s=.07, wrist_lag_s=.15,
                                                                                    key_hand_z=.007)),
                     ("fitted lags, key hand riding 10 mm high", dict(arm_lag_s=.07, wrist_lag_s=.15, key_hand_z=.010)),
                     ("fitted lags, key hand 3 mm off toward the low notes", dict(arm_lag_s=.07, wrist_lag_s=.15,
                                                                                  key_hand_y=.003)),
                     ("fitted lags, key hand 3 mm off toward the high notes", dict(arm_lag_s=.07, wrist_lag_s=.15,
                                                                                   key_hand_y=-.003)),
                     ("no_pump", dict(variant="no_pump")), ("pump_only", dict(variant="pump_only")),
                     ("prime_then_stop", dict(variant="prime_then_stop"))):
        report = dry_run(verbose=args.verbose, **kw)
        report.pop("phases")
        report.pop("frames")
        report.pop("stroke_ticks")
        report.pop("right_tcp")
        print(name, "->", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in report.items()}, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
