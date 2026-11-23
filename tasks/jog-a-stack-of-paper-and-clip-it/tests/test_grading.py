"""The scorer on constructed end states: every score value, every condition that zeroes a trial, the lift check."""

import numpy as np
import pytest

from rdb_paper_jog import grading as g
from rdb_paper_jog import paper_model as m

AWAY = np.array(((0., .30, .30), (0., -.30, .30)))
MIDDLE = np.array((.63, -.15, m.BLOCK[2]))


def stack(shifts=None, turn=0.):
    """Nine plates lying flat, long side along y; ``shifts``: per-plate (along the short side, along the long side)."""
    shifts = shifts or [(0., 0.)] * m.K
    return [(MIDDLE + (du, dv, (i + .5) * m.PLATE_T), m.rot_z(turn)) for i, (du, dv) in enumerate(shifts)]


def clip_on(poses, *, along=0., past=.015, side=1., opening=None, long_edge=False):
    """The clip pushed onto the stack's short edge at +v (``side`` 1) or -v, ``along`` from the edge's middle,
    its lips ``past`` the edge of the plate that sticks out most."""
    rotation = np.asarray(poses[0][1])
    u_axis, v_axis, w_axis = rotation[:, 0], rotation[:, 1], rotation[:, 2]
    middle = np.mean([p for p, _ in poses], axis=0)
    if long_edge:
        reach = max((m.corners(p, r) @ u_axis).max() for p, r in poses)
        jaw, width = -u_axis, v_axis
        lips = middle + (reach - middle @ u_axis - past) * u_axis + along * v_axis
    else:
        reach = max((side * (m.corners(p, r) @ v_axis)).max() for p, r in poses)
        jaw, width = -side * v_axis, side * u_axis
        lips = middle + side * (reach - side * (middle @ v_axis) - past) * v_axis + along * u_axis
    frame = np.column_stack((np.cross(width, jaw), width, jaw))
    gap = m.STACK_T if opening is None else opening
    return {"origin": lips - m.JAW_L * jaw, "rotation": frame, "opening": 2 * np.arcsin(gap / 2 / m.JAW_L), "lip_gap": gap}


def run(poses, clip, *, tcp=AWAY, touching=False, ticks=g.HOLD_TICKS, held=None, last=True):
    """Hold a state for ``ticks`` control steps and answer the lift check with ``held`` (per plate, default all)."""
    grade = g.PaperGrade()
    for tick in range(ticks):
        result = grade.update(poses, clip, tcp, touching, last=last and tick == ticks - 1)
    if result["wants_lift"]:
        result = grade.lifted(result, [True] * m.K if held is None else held)
    return result


def test_squared_clipped_and_let_go_is_full_score():
    poses = stack()
    result = run(poses, clip_on(poses))
    assert result["score"] == 1. and result["sheets_in_jaws"] == 36 and result["held_sheets"] == 36
    assert result["clip_placed"] and result["let_go"] and result["lift_checked"]


def test_full_score_needs_two_seconds_let_go_and_the_lift_check():
    poses = stack()
    early = run(poses, clip_on(poses), ticks=g.HOLD_TICKS - 1)
    assert early["score"] == .3 and not early["let_go"] and not early["lift_checked"]


@pytest.mark.parametrize("shifts, score", [
    ([(0., 0.)] * 8 + [(.0019, 0.)], 1.), ([(0., 0.)] * 8 + [(.004, 0.)], .6), ([(0., 0.)] * 8 + [(0., .004)], .6),
    ([(0., 0.)] * 8 + [(.0061, 0.)], .3), ([(0., 0.)] * 8 + [(0., .008)], .3)])
def test_flushness_tiers(shifts, score):
    poses = stack(shifts)
    assert run(poses, clip_on(poses))["score"] == score


def test_clip_off_the_middle_or_on_a_long_edge_is_point_six():
    poses = stack()
    assert run(poses, clip_on(poses, along=.019))["score"] == 1.
    off = run(poses, clip_on(poses, along=.030))
    assert off["score"] == .6 and not off["clip_placed"]
    long_edge = run(poses, clip_on(poses, long_edge=True))
    assert long_edge["score"] == .6 and not long_edge["clip_placed"] and long_edge["sheets_in_jaws"] == 36


def test_either_short_edge_will_do():
    poses = stack()
    assert run(poses, clip_on(poses, side=-1.))["score"] == 1.


def test_not_let_go_caps_at_point_three():
    poses = stack()
    near = np.array((MIDDLE + (0., 0., .040), (0., -.30, .30)))
    held = run(poses, clip_on(poses), tcp=near)
    assert held["score"] == .3 and not held["let_go"] and not held["wants_lift"]
    assert run(poses, clip_on(poses), touching=True)["score"] == .3


def test_lift_check_decides_how_many_sheets_count():
    poses = stack()
    clip = clip_on(poses)
    assert run(poses, clip, held=[True] * 8 + [False])["score"] == .3          # 32 held: not all
    assert run(poses, clip, held=[True] * 5 + [False] * 4)["score"] == .3      # 20 held: at least half
    assert run(poses, clip, held=[True] * 4 + [False] * 5)["score"] == 0.      # 16 held: fewer than half
    assert run(poses, clip, held=[False] * 9)["score"] == 0.


def test_clip_not_on_the_stack_or_not_shut_scores_nothing():
    poses = stack()
    parked = m.clip_state(*m.clip_start().values())
    assert run(poses, parked)["score"] == 0.
    held_open = clip_on(poses, opening=m.JAW_GAP_OPEN)                         # lips round the stack but not shut on it
    result = run(poses, held_open)
    assert result["sheets_in_jaws"] == 0 and result["score"] == 0.


def test_sheets_too_far_back_are_not_in_the_jaws():
    shifts = [(0., 0.)] * 5 + [(0., -.016)] * 4                                # four plates stop 1 mm short of the lips
    poses = stack(shifts)
    result = run(poses, clip_on(poses[:5], past=.015, opening=5 * m.PLATE_T))  # the lips shut on the five they reach
    assert result["sheets_in_jaws"] == 20 and result["score"] == .3
    assert run(poses, clip_on(poses, past=.015))["sheets_in_jaws"] == 0        # lips 1.6 mm wider than the five: not shut on them
    barely = [(0., 0.)] * 5 + [(0., -.013)] * 4                                # the other four are 2 mm under the lips
    poses = stack(barely)
    result = run(poses, clip_on(poses, past=.015))                             # lips shut on all nine; five far enough in to count
    assert result["sheets_in_jaws"] == 20 and result["score"] == .3


def test_turned_sheets_under_part_of_the_lips_still_count_as_holding_them_open():
    """A skewed stack: sheets turned a few degrees reach under one end of the lips but not under their middle.
    They hold the lips apart all the same, and the lips are shut on the paper."""
    poses = stack()
    turn = m.rot_z(np.radians(6.))
    back = np.array((0., -.0055, 0.))                         # their edge passes 0.7 mm short of the lips' middle, 1 mm under one end
    poses = poses[:5] + [(p + back, turn) for p, _ in poses[5:]]
    clip = clip_on(poses[:5], past=.004)
    clip = dict(clip, origin=clip["origin"] + (0., 0., 2 * m.PLATE_T), lip_gap=m.STACK_T, opening=2 * np.arcsin(m.STACK_T / 2 / m.JAW_L))
    middle = g.lips_middle(clip)
    assert all(m.SHEET_V / 2 - abs(g.in_plate(middle, pose)[1]) < 0. for pose in poses[5:])     # not under the middle
    assert g.in_jaws(poses, clip) == [True] * 5 + [False] * 4                                    # five count, lips shut on nine


def test_sheets_set_back_in_between_keep_the_lips_apart():
    """Every other plate is set back out of the lips' reach. Rigid plates keep their spacing, so the lips stand a
    whole stack apart while resting on the five that reach them: shut on the paper, five plates in the jaws."""
    poses = stack([(0., 0. if i % 2 == 0 else -.030) for i in range(m.K)])
    clip = clip_on(poses, past=.015)                                           # lip gap: the whole stack's thickness
    assert g.in_jaws(poses, clip) == [i % 2 == 0 for i in range(m.K)]
    assert run(poses, clip, held=[i % 2 == 0 for i in range(m.K)])["score"] == .3
    assert g.in_jaws(poses, dict(clip, lip_gap=m.JAW_GAP_OPEN)) == [False] * m.K          # held open round it: not shut


def test_one_lift_check_each_time_the_stack_is_let_go():
    poses = stack()
    clip = clip_on(poses)
    grade = g.PaperGrade()
    for _ in range(g.HOLD_TICKS):
        result = grade.update(poses, clip, AWAY, False)
    assert result["wants_lift"]
    assert grade.lifted(result, [True] * 8 + [False])["score"] == .3            # the candidate did not hold every sheet
    again = grade.update(poses, clip, AWAY, False)
    assert not again["wants_lift"] and again["score"] == .3                     # not lifted again at every step
    assert grade.update(poses, clip, AWAY, False, last=True)["wants_lift"]      # but once more at the end
    grade.update(poses, clip, AWAY, True)                                       # touched again
    for _ in range(g.HOLD_TICKS):
        result = grade.update(poses, clip, AWAY, False)
    assert result["wants_lift"]


def test_a_sheet_off_the_table_zeroes_the_trial_for_good():
    poses = stack()
    clip = clip_on(poses)
    grade = g.PaperGrade()
    fallen = [(p + (0., 0., -.2), r) if i == 3 else (p, r) for i, (p, r) in enumerate(poses)]
    assert grade.update(fallen, clip, AWAY, False)["score"] == 0.
    for _ in range(g.HOLD_TICKS):
        result = grade.update(poses, clip, AWAY, False, last=True)
    assert result["score"] == 0. and result["zeroed"] == "a sheet left the table" and not result["wants_lift"]
    over_edge = [(p + (.5, 0., 0.), r) for p, r in poses]
    assert g.fell(over_edge)


def test_temporary_fanning_does_not_invalidate_the_final_stack():
    poses = stack()
    clip = clip_on(poses)
    fanned = list(poses)
    fanned[-1] = (poses[-1][0], m.rot_y(np.radians(50.)))
    grade = g.PaperGrade()
    assert not grade.update(fanned, clip, AWAY, True)["zeroed"]
    for _ in range(g.HOLD_TICKS):
        result = grade.update(poses, clip, AWAY, False, last=True)
    assert result["wants_lift"]
    assert grade.lifted(result, [True] * m.K)["score"] == 1.


def test_nonfinite_state_scores_nothing():
    poses = stack()
    broken = [(poses[0][0] * np.nan, poses[0][1])] + poses[1:]
    result = g.PaperGrade().update(broken, clip_on(poses), AWAY, False)
    assert result["score"] == 0. and "error" in result


def test_start_state_scores_nothing_and_is_measured_as_skewed():
    poses = list(m.start_poses().values())
    result = run(poses, m.clip_state(*m.clip_start().values()))
    assert result["score"] == 0. and result["long_edges_mm"] > 15. and result["short_edges_mm"] > 15.


def test_flushness_does_not_depend_on_where_the_stack_lies():
    shifts = [(.001 * i, -.0005 * i) for i in range(m.K)]
    flat = m.misalignment(stack(shifts))
    assert flat == pytest.approx((.008, .004), abs=1e-9)
    turn = m.rot_y(1.1) @ m.rot_z(.4)
    moved = [(turn @ p + (.1, .2, .3), turn @ r) for p, r in stack(shifts)]
    assert m.misalignment(moved) == pytest.approx(flat, abs=1e-9)
    skewed = stack()
    skewed[4] = (skewed[4][0], m.rot_z(np.radians(1.)))
    long_side, short_side = m.misalignment(skewed)
    assert long_side == pytest.approx(m.SHEET_V / 2 * np.sin(np.radians(1.)), rel=.05) and short_side < long_side


def test_lift_check_carries_the_sheets_the_clip_holds():
    """A stand-in world: the clip follows the pull, the held plates follow the clip, the others stay."""
    for holding in ([True] * 9, [True] * 6 + [False] * 3):
        poses = stack()
        clip = clip_on(poses)
        check = g.LiftCheck(poses, clip, 1. / 240)
        velocity, steps = np.zeros(3), 0
        while True:
            force = check.force(poses, clip, velocity)
            if force is None:
                break
            assert 0. <= force[2] <= g.LIFT_MAX_N
            velocity = np.array((0., 0., min(g.LIFT_SPEED, velocity[2] + .01)))        # the clip comes up at the commanded speed
            if check.reached:
                velocity = np.zeros(3)
            step = velocity / 240
            clip = dict(clip, origin=clip["origin"] + step)
            poses = [(p + step, r) if hold else (p, r) for (p, r), hold in zip(poses, holding)]
            if not all(holding) and steps > 600:                                         # held plates hang clear of the table
                poses = [(p + (0., 0., .001), r) if not hold and p[2] < .4 else (p, r) for (p, r), hold in zip(poses, holding)]
            steps += 1
        if all(holding):
            assert check.reached and check.held(poses, clip) == holding
            assert check.time == pytest.approx(g.LIFT_HOLD_S + g.LIFT_CLEAR_M / g.LIFT_SPEED, abs=1.)


def test_lift_check_that_never_clears_the_table_holds_nothing():
    poses = stack()
    clip = clip_on(poses)
    check = g.LiftCheck([(p - (0., 0., m.BLOCK[2]), r) for p, r in poses], dict(clip, origin=clip["origin"] - (0., 0., m.BLOCK[2])), 1. / 240)
    low = [(p - (0., 0., m.BLOCK[2]), r) for p, r in poses]
    low_clip = dict(clip, origin=clip["origin"] - (0., 0., m.BLOCK[2]))
    while check.force(low, low_clip, np.zeros(3)) is not None:
        pass
    assert not check.reached and check.held(low, low_clip) == [False] * m.K
    assert check.time <= g.LIFT_TIMEOUT_S + g.LIFT_HOLD_S + .01
