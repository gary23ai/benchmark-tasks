"""On the table lies a portable pull-up projector screen in its closed case (a 20 in, 4:3 tabletop screen, 600 mm
long), square to the robot. The screen is rolled up inside on a spring roller; only its aluminium top bar shows, lying
along the case's top slot, with a raised handle and, at its left end, a raised tab. At the case's left end a
telescoping support post stands retracted, 4 cm proud of the case, with a short pin on top and a red pull knob.

Task: put the screen up. The screen's spring pulls it back into the case, and a support post holds it at full height. In the reference the bar is lifted while the post
is extended until it clicks. Once lowered, the top bar's tab drops over the pin on top of
the post and the screen hangs there, fully extended. Let go of the screen early and it rolls back into the case; let go
of the post before it clicks and it slides back down.

Run it:  inspect-robots run --embodiment yam_arena -E task=put_up_a_tabletop_pull_up_projector_screen.py \\
           --instruction "..." --scorer task_score ...

Mechanism toolkit: ``bodykit.py``, unchanged, from the accepted task "Reload a long-arm stapler and staple two sheets
together" in robocurve/benchmark-tasks (credited in README.md).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from screen_checks import distance_to_segment, release_time
import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # score() counts the complete two-second interval itself.

# -- the screen (world frame: x away from the arm bases, y to the robot's left, z up; heights above the table; m) -------
CASE_X = (0.400, 0.480)  # the case, front to back
CASE_HALF = 0.300  # half its length (y)
CASE_H = 0.085
PLANE_X = 0.440  # the screen's plane (the case's slot)
SCREEN_HALF = 0.205  # a 20 in 4:3 screen: 410 wide
SCREEN_UP = 0.305  # and 305 high: pulled out this far, it is fully up
SCREEN_K = 6.0  # N/m: the roller spring pulls the screen back in with about 5 N, 7 N fully up
SCREEN_PRE = 0.80  # (it pulls toward this far below its rest position)
SCREEN_MASS = 0.35  # the top bar and the screen
BAR = (0.020, 2 * SCREEN_HALF + 0.020, 0.016)  # the top bar (x, y, z)
HANDLE_Y = -0.080  # the bar's raised handle (a rod 30 mm above the bar, 70 mm long)
HANDLE_UP, HANDLE_R, HANDLE_L = 0.030, 0.006, 0.070
RISER = 0.046  # the tab stands on a riser this high above the bar's top
POST_Y = 0.245  # the support post's axis (y), at the case's left end, in the screen's plane
POST_R = 0.009
POST_TOP0_UP = 0.132  # the post's top above the table at rest (4 cm proud of the case)
POST_CLICK = 0.003  # pulled up to within this of its full travel, it clicks and stays
POST_FALL = (6.0, 0.30)  # N/m, m: unclicked, a light spring pulls it back down (about 2 N)
PIN_R, PIN_H = 0.004, 0.008
KNOB_R = 0.010
KNOB_Y = 0.030  # the pull knob sticks out on the post's left, this far from its axis
TAB = (0.024, 0.040, 0.004)  # the tab over the post (x, y, z), with a 12 mm hole round the pin's line
HOLE = 0.012

# -- scoring --------------------------------------------------------------------------------------------------------------
UP_TOL = 0.010  # "fully up": the top bar within 10 mm of its fully-up height
HUNG_TOL = 0.004  # hung: the tab resting on the post's top (within 4 mm), round its pin
CLEAR_M = 0.050  # both gripper centre points at least this far from the top bar and the post

ALU, DARK, WHITE, GREY, RED = (0.78, 0.79, 0.81), (0.10, 0.10, 0.11), (0.96, 0.96, 0.95), (0.55, 0.56, 0.58), \
    (0.75, 0.10, 0.08)


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


def _case() -> Body:
    """The case, on its rubber feet (it does not slide); origin on the table under the slot's middle."""
    x0, x1 = CASE_X[0] - PLANE_X, CASE_X[1] - PLANE_X
    gap = 0.006  # the slot
    shapes = [_box(x0, -gap / 2, -CASE_HALF, CASE_HALF, 0.0, CASE_H, DARK),
              _box(gap / 2, x1, -CASE_HALF, CASE_HALF, 0.0, CASE_H, DARK),
              _box(-gap / 2, gap / 2, -CASE_HALF, CASE_HALF, 0.0, 0.010, DARK),  # the slot's floor
              *[_box(x0 + 0.004, x1 - 0.004, s * CASE_HALF - 0.006, s * CASE_HALF + 0.006, CASE_H, CASE_H + 0.004, GREY)
                for s in (-1.0, 1.0)]]  # end caps
    return Body.build("case", pos=(PLANE_X, 0.0, TOP), mass=2.5, shapes=shapes, kinematic=True, color=DARK)


SCREEN_REST = np.array([PLANE_X, 0.0, TOP + CASE_H + BAR[2] / 2])  # the top bar's middle at rest


def _screen() -> Body:
    """The top bar with its handle and its tab (these collide), and the screen hanging under the bar (drawn only: it
    slides down into the case and the table, out of sight, when rolled up). Origin at the bar's middle."""
    top = BAR[2] / 2
    shapes = [BoxShape((0.0, 0.0, 0.0), BAR, ALU),
              BoxShape((0.0, 0.0, -top - SCREEN_UP / 2), (0.002, 2 * SCREEN_HALF, SCREEN_UP), WHITE, collide=False),
              CylShape((0.0, HANDLE_Y, top + HANDLE_UP), HANDLE_R, HANDLE_L, DARK, axis="Y"),  # the handle
              *[_box(-0.004, 0.004, HANDLE_Y + s * HANDLE_L / 2 - 0.004, HANDLE_Y + s * HANDLE_L / 2 + 0.004, top,
                     top + HANDLE_UP, DARK) for s in (-1.0, 1.0)]]
    # the tab: an arm from a riser at the bar's left end out over the post, a square hole round the pin's line
    ty, tz, h = POST_Y, top + RISER + TAB[2] / 2, HOLE / 2
    end = SCREEN_HALF + 0.010  # the bar's left end
    shapes += [_box(-0.004, 0.004, end - 0.010, end, top, top + RISER, ALU),  # the riser
               _box(-TAB[0] / 2, TAB[0] / 2, end - 0.010, ty - h, tz - TAB[2] / 2, tz + TAB[2] / 2, ALU),
               _box(-TAB[0] / 2, TAB[0] / 2, ty + h, ty + TAB[1] / 2, tz - TAB[2] / 2, tz + TAB[2] / 2, ALU),
               _box(-TAB[0] / 2, -h, ty - h, ty + h, tz - TAB[2] / 2, tz + TAB[2] / 2, ALU),
               _box(h, TAB[0] / 2, ty - h, ty + h, tz - TAB[2] / 2, tz + TAB[2] / 2, ALU)]
    return Body.build("screen", pos=tuple(SCREEN_REST.tolist()), mass=SCREEN_MASS, shapes=shapes, color=ALU,
                      friction=(0.6, 0.5), ignore=("case",), solver_iterations=32, contact_offset=0.001)


TAB_UNDER0 = SCREEN_REST[2] + BAR[2] / 2 + RISER  # the tab's underside at rest (world z)
POST_TOP0 = TOP + POST_TOP0_UP  # the post's top (under its pin) at rest
POST_UP = float(TAB_UNDER0 + SCREEN_UP - POST_TOP0)  # its travel to its click: its top then meets the fully-up tab
POST_REST = np.array([PLANE_X, POST_Y, POST_TOP0])


def _post() -> Body:
    """Origin at the post's top (where the tab comes to rest). The tube runs down into the case."""
    shapes = [CylShape((0.0, 0.0, -0.015), POST_R, 0.030, ALU),  # its top section (collides)
              CylShape((0.0, 0.0, -0.21), POST_R - 0.001, 0.36, ALU, collide=False),  # the rest of it, down into the case
              CylShape((0.0, 0.0, PIN_H / 2), PIN_R, PIN_H, GREY),
              CylShape((0.0, KNOB_Y, -0.004), KNOB_R, 0.016, RED, axis="Y")]  # the pull knob on its left
    return Body.build("post", pos=tuple(POST_REST.tolist()), mass=0.15, shapes=shapes, color=ALU,
                      friction=(0.7, 0.6), ignore=("case",), solver_iterations=32, contact_offset=0.001)


OBJECTS = [_case(), _screen(), _post()]

# the screen's roller spring: the top bar slides up out of the slot (0 at rest), pulled back in
slider("roller", "case", "screen", anchor=tuple(SCREEN_REST.tolist()), axis="Z", limits=(0.0, SCREEN_UP + 0.04),
       stiffness=SCREEN_K, damping=3.0, target=-SCREEN_PRE, max_force=40.0)
# the post's slide: up to its click (POST_UP), pulled back down until it clicks
slider("postslide", "case", "post", anchor=tuple(POST_REST.tolist()), axis="Z", limits=(0.0, POST_UP),
       stiffness=POST_FALL[0], damping=4.0, target=-POST_FALL[1], max_force=40.0)


# -- followed after every control step -----------------------------------------------------------------------------------


def _fresh() -> dict:
    return {"bar_up": 0.0, "post_up": 0.0, "clicked": False, "tcps": None, "ever_clicked": False, "ever_up": False,
            "ever_both": False, "hung": False, "dbg": 0, "release_start": None, "released_s": 0.0, "elapsed": 0.0}


_EP = _fresh()
_DEBUG = bool(os.environ.get("SCREEN_DEBUG"))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _pose(base, origin, name):
    t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return t[:3] - origin, rotation(t[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    s_pos, _sr = _pose(base, origin, "screen")
    p_pos, _pr = _pose(base, origin, "post")
    bar_up = float(s_pos[2] - SCREEN_REST[2])
    post_up = float(p_pos[2] - POST_REST[2])
    _EP["bar_up"], _EP["post_up"] = bar_up, post_up
    _EP["tcps"] = _tcps(base, origin)
    if not _EP["clicked"] and post_up >= POST_UP - POST_CLICK:  # the post clicks at its full height and stays
        _EP["clicked"] = True
        set_slider_drive("postslide", target=POST_UP, stiffness=5000.0)
    tab_under = float(s_pos[2] + BAR[2] / 2 + RISER)  # the tab's underside
    _EP["hung"] = _EP["clicked"] and abs(tab_under - float(p_pos[2])) <= HUNG_TOL and \
        abs(bar_up - SCREEN_UP) <= UP_TOL
    up = abs(bar_up - SCREEN_UP) <= UP_TOL or bar_up > SCREEN_UP
    _EP["ever_clicked"] = _EP["ever_clicked"] or _EP["clicked"]
    _EP["ever_up"] = _EP["ever_up"] or up
    _EP["ever_both"] = _EP["ever_both"] or (up and _EP["clicked"])
    _EP["elapsed"] += float(base.step_dt)
    _EP["release_start"], _EP["released_s"] = release_time(
        _EP["hung"] and _clear(), _EP["elapsed"], _EP["release_start"])
    if _DEBUG:
        _EP["dbg"] += 1
        if _EP["dbg"] % 10 == 1:
            tl, tr = _EP["tcps"]
            print(f"[screen] t={_EP['dbg'] / 20:5.1f}s bar_up={bar_up * 1000:6.1f}mm post_up={post_up * 1000:6.1f}mm "
                  f"clicked={_EP['clicked']} hung={_EP['hung']} L={np.round(tl * 1000).astype(int).tolist()} "
                  f"R={np.round(tr * 1000).astype(int).tolist()} clear={_clear()} score={score(None)}",
                  file=sys.stderr, flush=True)


def bar_point(up: float, y: float = 0.0) -> np.ndarray:
    return SCREEN_REST + np.array([0.0, y, up])


def knob_point(up: float) -> np.ndarray:
    return POST_REST + np.array([0.0, KNOB_Y, -0.004 + up])


def handle_point(up: float) -> np.ndarray:
    return SCREEN_REST + np.array([0.0, HANDLE_Y, BAR[2] / 2 + HANDLE_UP + up])


def _clear() -> bool:
    if _EP["tcps"] is None:
        return False
    bar = bar_point(_EP["bar_up"])
    post = POST_REST + [0.0, 0.0, _EP["post_up"]]
    knob = knob_point(_EP["post_up"])
    bar_half = BAR[1] / 2
    lines = [(bar - [0, bar_half, 0], bar + [0, bar_half, 0]),
             (np.array([PLANE_X, POST_Y, TOP + CASE_H]), post + [0, 0, PIN_H]),
             (post, knob)]
    return all(distance_to_segment(tcp, start, end) >= CLEAR_M
               for tcp in _EP["tcps"] for start, end in lines)



# -- scoring --------------------------------------------------------------------------------------------------------


def score(objects) -> float:
    """Read live. 1.0: the screen hung fully up (the post clicked at its full height, the top bar's tab resting on
    the post's top round its pin, the top bar within 10 mm of fully up) with both gripper centre points at least
    50 mm from the top bar and the post (held 2 s for success). 0.5: at some moment the post clicked with the screen
    fully up (or higher). 0.25: at some moment the post clicked, or the screen fully up. 0.0 otherwise."""
    if _EP["released_s"] >= 2.0 - 1e-9:
        return 1.0
    if _EP["ever_both"]:
        return 0.5
    return 0.25 if (_EP["ever_clicked"] or _EP["ever_up"]) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- scripted demo ------------------------------------------------------------------------------------------------------
# The right hand takes the top bar from the front, level, fingers above and below it, and pulls the screen straight
# up a little past fully up, and holds it. The left hand takes the post's knob the same way, pulls the post up until
# it clicks, and lets go. The right lets the screen down onto the post (its tab drops over the pin), opens and goes
# back; the left goes back.


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


LEVEL = _ry(-90.0) @ grip_rotation(math.pi / 2)  # pointing forward (+x), level, fingers closing up-down
LEAN20 = _ry(-70.0) @ grip_rotation(math.pi / 2)  # the same, pointing 20 degrees down (reaches the low knob)
OPEN = -0.030
BAR_Y = -0.10  # the right hand holds the bar here
OVER = 0.060  # aim this far past fully up while the post goes up (the arm lags under the spring's pull up there)
POST_OVER = 0.045  # aim this far past the post's click (the arm lags about 3 cm up there; the post stops at its click)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(secs / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _right(clicked_at: float):
    b0 = handle_point(0.0)
    out = _lead_in("right", b0 + [-0.06, 0.0, 0.05], LEVEL, "in front of the screen's top bar, above it", OPEN)
    out.append(_w(1.0, b0 - [0.05, 0.0, 0.0], OPEN, "down in front of it", LEVEL))
    out += [_w(1.0, b0, OPEN, "fingers round the bar's handle, above and below", LEVEL),
            _w(0.6, b0, FINGER_CLOSED, "take it", LEVEL)]
    high = handle_point(SCREEN_UP + OVER)
    out.append(_w(3.5, high, FINGER_CLOSED, "pull the screen straight up, a little past fully up", LEVEL))
    up_at = sum(w.duration for w in out)
    t = up_at
    out.append(_w(max(clicked_at + 0.8 - t, 0.5), high, FINGER_CLOSED, "hold it there while the post goes up", LEVEL))
    rest = handle_point(SCREEN_UP)
    out += [_w(1.5, rest + [0.0, 0.0, 0.002], FINGER_CLOSED, "let it down onto the post", LEVEL),
            _w(0.6, rest + [0.0, 0.0, 0.002], OPEN, "let go", LEVEL),
            _w(1.0, rest - [0.07, 0.0, 0.0], OPEN, "back", LEVEL)]
    out += _go_home("right", rest - [0.07, 0.0, 0.0], LEVEL, OPEN)
    return out, up_at


def _left(up_at: float, total: float):
    k0 = knob_point(0.0)
    pre = k0 - LEAN20 @ np.array([0.0, 0.0, 0.06])  # 6 cm back along the gripper's pointing direction
    out = _lead_in("left", pre + [0.0, 0.0, 0.04], LEAN20, "in front of the post's knob, above it", OPEN)
    t = sum(w.duration for w in out)
    out.append(_w(max(up_at - t - 1.6, 0.5), pre, OPEN, "wait for the screen", LEAN20))
    out += [_w(1.0, k0, OPEN, "fingers round the knob", LEAN20), _w(0.6, k0, FINGER_CLOSED, "take it", LEAN20)]
    top = knob_point(POST_UP + POST_OVER)
    extended_test = bool(os.environ.get("SCREEN_POST_SLOW"))
    lift_seconds = 5.0 if extended_test else 3.0
    out.append(_w(lift_seconds, top, FINGER_CLOSED, "pull the post up until it clicks", LEAN20))
    if extended_test:
        out.append(_w(3.0, top, FINGER_CLOSED, "hold the knob at full extension", LEAN20))
    clicked_at = sum(w.duration for w in out)
    back = top - LEAN20 @ np.array([0.0, 0.0, 0.07])
    out += [_w(0.6, top, OPEN, "let go", LEAN20), _w(1.0, back, OPEN, "back", LEAN20)]
    out += _go_home("left", back, LEAN20, OPEN)
    t = sum(w.duration for w in out)
    out.append(Waypoint(max(total - t, 0.5), out[-1].pos, OPEN, "hold still", out[-1].yaw))
    return out, clicked_at


def demo():
    right0, up_at = _right(0.0)
    left0, clicked_at = _left(up_at, 0.0)
    right, _ = _right(clicked_at)
    total = max(sum(w.duration for w in right), sum(w.duration for w in left0)) + 4.0
    left, _ = _left(up_at, total)
    right.append(Waypoint(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, OPEN, "hold still",
                          right[-1].yaw))
    if os.environ.get("SCREEN_ONE_HAND"):  # the one-hand check: the left stays where it started
        from yam_arena import ik, station
        from yam_arena.constants import READY

        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        left = [Waypoint(total, tuple(float(v) for v in p), FINGER_CLOSED, "idle at the start", r)]
    if os.environ.get("SCREEN_POST_ONLY"):
        from yam_arena import ik, station
        from yam_arena.constants import READY
        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
        right = [Waypoint(total, tuple(float(v) for v in p), OPEN, "idle at the start", r)]
    return {"left": left, "right": right}


if __name__ == "__main__":
    print({"post_up_m": round(POST_UP, 3), "handle_rest": np.round(handle_point(0.0), 3).tolist(), "handle_up": np.round(handle_point(SCREEN_UP), 3).tolist(),
           "knob_rest": np.round(knob_point(0.0), 3).tolist(), "knob_up": np.round(knob_point(POST_UP), 3).tolist(),
           "screen_pull_n": (round(SCREEN_K * SCREEN_PRE, 1), round(SCREEN_K * (SCREEN_PRE + SCREEN_UP), 1))})
    plan = demo()
    print({arm: (len(w), round(sum(x.duration for x in w), 1)) for arm, w in plan.items()})
