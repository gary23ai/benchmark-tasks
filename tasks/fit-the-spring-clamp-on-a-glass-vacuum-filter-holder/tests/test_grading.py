"""Every scoring tier and the cheap strategies, on hand-built states (NumPy only)."""

import numpy as np

from rdb_filter_clamp import fixed_scene as F
from rdb_filter_clamp import grading as G
from rdb_filter_clamp import layout as L

AXIS = np.array((*L.AXIS, 0.))
JOINT = AXIS + (0., 0., L.JOINT_Z)
I3 = np.eye(3)
FAR = [np.array((.10, .25, .30)), np.array((.10, -.25, .30))]


def tilt_x(deg):
    t = np.radians(deg)
    return np.array(((1., 0., 0.), (0., np.cos(t), -np.sin(t)), (0., np.sin(t), np.cos(t))))


def state(funnel=JOINT, funnel_rot=I3, clamp=None, jaws=(1.4, 1.4), clamp_rot=I3, tcps=FAR, openings=(90., 90.),
          holder=AXIS, holder_rot=I3):
    return {"holder_pos": holder, "holder_rot": holder_rot, "funnel_pos": np.asarray(funnel, float), "funnel_rot": funnel_rot,
            "clamp_pos": F.CLAMP_START if clamp is None else np.asarray(clamp, float), "clamp_rot": clamp_rot,
            "jaw_open_deg": list(jaws), "tcp": tcps, "tcp_rot": [I3, I3], "opening_mm": list(openings)}


def holding_left(funnel=JOINT):
    return [np.asarray(funnel) + (0., 0., .065), FAR[1]], (64., 90.)


def grade():
    return G.FilterClampGrade(holder_start=tuple(F.HOLDER_START), clamp_start=tuple(F.CLAMP_START))


def run(g, states):
    out = None
    for s in states:
        out = g.update(s)
    return out


def full_trial(hold=True, approach_err=(0., 0.), slide=0.):
    """Funnel seated and held, clamp pushed on from behind, closed; then hands off."""
    seq = []
    tcps, op = holding_left()
    for _ in range(10):
        seq.append(state(tcps=tcps if hold else FAR, openings=op if hold else (90., 90.)))
    for d in np.linspace(.08, 0., 20):           # the open clamp comes in from behind
        c = JOINT + (-d + approach_err[0], approach_err[1], 0.)
        f = JOINT + (slide * (1 - d / .08), 0., 0.)
        t, o = holding_left(f)
        seq.append(state(funnel=f, clamp=c, jaws=(27., 27.), tcps=t if hold else FAR, openings=o if hold else (90., 90.)))
    f = JOINT + (slide, 0., 0.)
    t, o = holding_left(f)
    for _ in range(12):
        seq.append(state(funnel=f, clamp=JOINT, tcps=t if hold else FAR, openings=o if hold else (90., 90.)))
    for _ in range(30):
        seq.append(state(funnel=f, clamp=JOINT))
    return seq


def test_full_trial_scores_one_and_needs_one_second_still():
    g = grade()
    seq = full_trial()
    out = run(g, seq[:-10])
    assert out["score"] == .8                     # not yet a second hands-off
    out = run(g, seq[-10:])
    assert out["score"] == 1. and g.finished


def test_tiers_in_order():
    assert run(grade(), [state(funnel=F.FUNNEL_START)] * 3)["score"] == 0.          # funnel still on the wipe
    assert run(grade(), [state(funnel=JOINT + (.003, 0., 0.))] * 3)["score"] == .2   # seated, off centre
    assert run(grade(), [state()] * 3)["score"] == .4                               # seated and centred
    assert run(grade(), [state(funnel=JOINT + (0., 0., .004))] * 3)["score"] == 0.   # held above the base


def test_funnel_tilted_is_not_seated():
    assert run(grade(), [state(funnel_rot=tilt_x(4.))] * 3)["score"] == 0.


def test_clamp_not_closed_or_off_the_joint_stays_at_04():
    t, o = holding_left()
    assert run(grade(), [state(clamp=JOINT, jaws=(27., 27.), tcps=t, openings=o)] * 5)["score"] == .4   # still open
    assert run(grade(), [state(clamp=JOINT, jaws=(-2., -2.), tcps=t, openings=o)] * 5)["score"] == .4   # shut on nothing
    assert run(grade(), [state(clamp=JOINT + (0., 0., .006), tcps=t, openings=o)] * 5)["score"] == .4   # too high


def test_clamp_fitted_while_nobody_held_the_funnel_caps_at_06():
    out = run(grade(), full_trial(hold=False))
    assert out["unheld_fitting"] and out["score"] == .6


def test_funnel_pushed_across_the_membrane_caps_at_06():
    out = run(grade(), full_trial(slide=.0013))
    assert out["membrane_dragged"] and out["membrane_offset_mm"] == 1.3 and out["score"] == .6
    out = run(grade(), full_trial(slide=.0008))          # under the 1 mm limit
    assert not out["membrane_dragged"] and out["score"] == 1.


def test_clamp_teleported_on_without_fitting_does_not_reach_08():
    g = grade()
    out = run(g, [state(clamp=JOINT)] * 40)
    assert out["score"] == .6                              # no fitting was ever watched


def test_still_holding_at_the_end_scores_08():
    seq = full_trial()[:-30]
    t, o = holding_left()
    out = run(grade(), seq + [state(clamp=JOINT, tcps=t, openings=o)] * 40)
    assert out["score"] == .8 and not out["clear"]


def test_unreadable_grippers_count_as_not_clear():
    seq = full_trial()[:-30] + [dict(state(clamp=JOINT), tcp=None, tcp_rot=None, opening_mm=None)] * 40
    assert run(grade(), seq)["score"] == .8


def test_flask_knocked_over_is_a_latched_zero():
    seq = full_trial()
    seq.insert(15, state(holder_rot=tilt_x(12.)))
    out = run(grade(), seq)
    assert out["failed"] == "flask knocked over" and out["score"] == 0.


def test_flask_pushed_along_the_table_is_a_latched_zero():
    seq = [state(holder=AXIS + (.04, 0., 0.), funnel=JOINT + (.04, 0., 0.))] + full_trial()
    assert run(grade(), seq)["failed"] == "flask pushed along the table"


def test_clamp_dropped_is_a_latched_zero():
    seq = full_trial()
    seq.insert(12, state(clamp=(.40, -.30, .005)))
    assert run(grade(), seq)["failed"] == "clamp dropped"


def test_lifting_and_resetting_the_funnel_is_a_new_placing():
    g = grade()
    run(g, [state(funnel=JOINT + (.003, 0., 0.))] * 3)
    run(g, [state(funnel=JOINT + (.003, 0., .006))] * 3)          # lifted 6 mm
    out = run(g, [state()] * 3)
    assert not out["membrane_dragged"] and out["score"] == .4


def test_hold_detection():
    assert G.held_by(JOINT + (0., 0., .065), I3, 64., JOINT, I3)
    assert not G.held_by(JOINT + (0., 0., .065), I3, 90., JOINT, I3)       # open pads
    assert not G.held_by(JOINT + (.03, 0., .065), I3, 64., JOINT, I3)      # beside the funnel
    assert not G.held_by(JOINT + (0., 0., .010), I3, 64., JOINT, I3)       # on the flange, not the body
    assert not G.held_by(None, I3, 64., JOINT, I3)


def test_a_funnel_moving_while_hovering_just_above_the_membrane_does_not_drag_it():
    g = grade()
    run(g, [state()] * 2)                                              # touches
    out = run(g, [state(funnel=JOINT + (.0015 * k / 5, 0., .0006)) for k in range(6)])   # 0.6 mm up, drifts 1.5 mm
    out = run(g, [state(funnel=JOINT + (.0015, 0., 0.))] * 2)          # set down 1.5 mm away: nothing dragged
    assert not out["membrane_dragged"] and out["membrane_offset_mm"] == 0.
    out = run(g, [state(funnel=JOINT + (.0015 - .0012 * k / 4, 0., 0.)) for k in range(5)])   # slid 1.2 mm on it
    assert out["membrane_dragged"] and out["membrane_offset_mm"] == 1.2


def test_the_membrane_is_judged_at_the_end_so_a_drag_that_returns_is_not_penalised():
    g = grade()
    run(g, [state(funnel=JOINT + (.002 * k / 5, 0., 0.)) for k in range(6)])     # slid 2 mm while resting on it
    assert g.last["membrane_dragged"]
    out = run(g, [state(funnel=JOINT + (.002 - .002 * k / 5, 0., 0.)) for k in range(6)])   # and back again
    assert not out["membrane_dragged"] and out["membrane_peak_mm"] == 2. and out["membrane_offset_mm"] == 0.
