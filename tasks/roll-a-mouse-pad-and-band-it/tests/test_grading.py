"""The scorer on hand-built end states: every tier, every way to lose a tier, and folding instead of rolling."""

import numpy as np

from kinematic_pad import FLAT, pad, roll_bends
from rdb_pad_roll import grading, pad_model as m

EYE = np.eye(3)
AWAY = [np.array((.10, .30, .30)), np.array((.10, -.30, .30))]       # both grippers far from everything
CRADLE = m.BAND_START
IN_BAND = roll_bends(1.3)         # a roll let go inside the 70 mm bore relaxes to about this
CENTRED = (m.PAD_W + m.BAND_L) / 2


def state(*, bends=IN_BAND, on=CENTRED, band=CRADLE, band_rot=EYE, tcp=AWAY, offsets=None, on_carton=False):
    """A pad with fold angles ``bends``. Lying along the band's axis, pushed ``on`` metres into the band from one
    of its ends (BAND_L: just through; CENTRED: the band round the roll's middle). ``on_carton``: where it starts
    instead."""
    if on_carton:
        strips, rotations = pad(bends, (m.PAD_X0 + m.PAD_L / 2, m.PAD_Y, m.PAD_Z), along=(0., 1., 0.))
    else:
        axis = band_rot[:, 0]
        strips, rotations = pad(bends, band + axis * (m.BAND_L / 2 - on + m.PAD_W / 2), along=axis,
                                up=band_rot[:, 2], offsets=offsets)
    return dict(strips=strips, strip_rot=rotations, band_pos=np.asarray(band, dtype=float), band_rot=band_rot, tcp=tcp)


def final(ticks=grading.CLEAR_TICKS + 3, **kwargs):
    grade = grading.PadRollGrade()
    for _ in range(ticks):
        result = grade.update(**state(**kwargs))
    return result


def test_full_marks_need_two_seconds_clear():
    grade = grading.PadRollGrade()
    scores = [grade.update(**state())["score"] for _ in range(grading.CLEAR_TICKS + 1)]
    assert scores[0] == .6 and scores[grading.CLEAR_TICKS - 2] == .6 and scores[-1] == 1.


def test_tiers():
    assert final(bends=FLAT, on_carton=True)["score"] == 0.                      # nothing done
    rolled = final(bends=m.tight_bends(), on_carton=True)
    assert rolled["pad_is_a_roll"] and rolled["score"] == 0.                      # rolled, but no band on it
    assert final(on=.010)["score"] == 0.                                          # the band barely started
    part = final(on=.040)
    assert part["score"] == .3 and not part["band_on_roll"]
    assert final()["score"] == 1.


def test_a_third_of_the_band_is_the_line():
    assert final(on=.30 * m.BAND_L)["score"] == 0.
    assert final(on=.37 * m.BAND_L)["score"] == .3


def test_roll_must_stand_out_of_both_ends_of_the_band():
    short = final(on=m.BAND_L + .006)                                             # 6 mm through: it could slip off
    assert short["roll_out_of_band_mm"] == 6. and short["score"] == .3
    assert final(on=m.BAND_L + .014)["score"] == 1.
    assert final(on=m.PAD_W - .006)["score"] == .3                                # the same at the roll's other end
    assert final(on=m.PAD_W - .014)["score"] == 1.


def test_telescoped_roll_loses_full_marks():
    spread = np.linspace(-.007, .007, m.STRIPS)                                   # 14 mm from the inside turn to the outside
    crooked = final(offsets=spread)
    assert crooked["turns_offset_mm"] == 14. and crooked["band_on_roll"] and crooked["score"] == .6
    assert final(offsets=np.linspace(-.004, .004, m.STRIPS))["score"] == 1.       # 8 mm is within the limit


def test_the_banded_roll_may_be_put_down_anywhere():
    assert final(band=np.array((.30, -.10, m.BAND_OD / 2)))["score"] == 1.        # on the table
    assert final(band=CRADLE + (0., 0., .06), tcp=[CRADLE + (0., 0., .06 + m.BAND_OD / 2 + .01), AWAY[1]])["score"] == .6   # still held up


def test_grippers_must_let_go_of_band_and_pad():
    holding = [CRADLE + (0., 0., m.BAND_OD / 2 + .01), AWAY[1]]
    assert final(tcp=holding)["score"] == .6
    near = [CRADLE + (0., 0., m.BAND_OD / 2 + .04), AWAY[1]]                      # 40 mm off: not yet clear
    assert final(tcp=near)["score"] == .6
    assert final(tcp=[CRADLE + (0., 0., m.BAND_OD / 2 + .06), AWAY[1]])["score"] == 1.
    on_the_pad = [CRADLE + (.09, 0., .03), AWAY[1]]                               # clear of the band, but holding the roll
    assert final(tcp=on_the_pad)["score"] == .6
    grade = grading.PadRollGrade()                                                # touching it again restarts the 2 s
    for _ in range(grading.CLEAR_TICKS + 3):
        grade.update(**state())
    assert grade.update(**state(tcp=holding))["score"] == .6
    assert grade.update(**state())["score"] == .6


def test_folded_and_banded_scores_zero():
    """Folding the pad instead of rolling it scores nothing, even where the folded pad goes through the band."""
    crease = np.radians(175.)
    zigzag = FLAT.copy()                              # five panels of five strips, folded back and forth: 50 x 15 mm
    for k, hinge in enumerate((5, 10, 15, 20)):
        zigzag[hinge - 1] = crease if k % 2 == 0 else -crease
    banded = final(bends=zigzag)
    assert banded["band_on_roll"], "the folded stack is small enough to go through the bore in this test"
    assert not banded["pad_is_a_roll"] and banded["score"] == 0.
    in_half = FLAT.copy()
    in_half[11] = crease                              # folded once
    assert final(bends=in_half)["score"] == 0.
    in_four = FLAT.copy()                             # folded in half twice the same way: one turn, not a roll
    in_four[11] = crease
    in_four[5] = in_four[17] = crease / 2
    assert final(bends=in_four)["turns"] < grading.MIN_TURNS and final(bends=in_four)["score"] == 0.


def test_a_roll_winds_one_way():
    bent_back = IN_BAND.copy()
    bent_back[12] = -np.radians(30.)                  # one place bent back on itself
    result = final(bends=bent_back)
    assert result["back_bend_deg"] == 30. and not result["pad_is_a_roll"] and result["score"] == 0.
    open_c = final(bends=roll_bends(.9))              # opened out to less than a full turn: not a roll any more
    assert not open_c["pad_is_a_roll"] and open_c["score"] == 0.
    assert final(bends=roll_bends(1.6), on=.040)["score"] == .3


def test_a_roll_too_fat_for_the_band_is_not_in_it():
    fat = roll_bends(1.02)                            # about 80 mm across: wider than the 70 mm bore
    result = final(bends=fat)
    assert result["pad_is_a_roll"] and result["outside_bore_mm"] > 1. and result["share_of_band_on_roll"] < 1.
    assert result["score"] < 1.


def test_pad_beside_the_band_is_not_in_it():
    beside = state()
    beside["strips"] = beside["strips"] + (0., .08, 0.)
    result = grading.PadRollGrade().update(**beside)
    assert result["share_of_band_on_roll"] == 0. and result["score"] == 0.


def test_doing_nothing_scores_zero_for_the_whole_trial():
    grade = grading.PadRollGrade()
    assert all(grade.update(**state(bends=FLAT, on_carton=True))["score"] == 0. for _ in range(60))


def test_loose_roll_requires_one_full_turn_even_if_the_band_is_present():
    almost = grading.PadRollGrade().update(**state(bends=roll_bends(.99)))
    complete = grading.PadRollGrade().update(**state(bends=roll_bends(1.086)))
    assert not almost["pad_is_a_roll"] and almost["score"] == 0.
    assert complete["pad_is_a_roll"]
