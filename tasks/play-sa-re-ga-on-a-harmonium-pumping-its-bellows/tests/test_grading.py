"""The scorer on sounding timelines: full score only for Sa, Re, Ga each held 4 s, in order and legato."""

import numpy as np

from rdb_harmonium import fixed_scene as fixture
from rdb_harmonium import grading
from rdb_harmonium.grading import PhraseGrade

HZ = grading.CONTROL_HZ
HOME = fixture.CASE_START


def frames(*parts):
    """parts: (key names or (), seconds) -> one tuple per 50 ms frame."""
    out = []
    for names, seconds in parts:
        out += [tuple(names)] * int(round(seconds * HZ))
    return out


def play(timeline, *, release_s=1.2, position=HOME, rotation=np.eye(3), touching=True):
    grade = PhraseGrade(start_xy=HOME[:2])
    result = None
    for f in timeline:
        result = grade.update(f, position, rotation, touching, keys_up=not f)
    for _ in range(int(round(release_s * HZ))):
        result = grade.update((), position, rotation, False, True)
    return result, grade


def phrase(join):
    """Sa, Re, Ga 4.4 s each; ``join`` > 0 a silence between notes, < 0 an overlap (legato)."""
    if join >= 0:
        return frames((("C4",), 4.4), ((), join), (("D4",), 4.4), ((), join), (("E4",), 4.4))
    o = -join
    return frames((("C4",), 4.4 - o), (("C4", "D4"), o), (("D4",), 4.4 - 2 * o), (("D4", "E4"), o),
                  (("E4",), 4.4 - o))


PHRASE = phrase(-.2)


def test_the_legato_phrase_scores_one():
    result, grade = play(PHRASE)
    assert result["score"] == 1. and grade.finished and result["joins_s"] == (-.2, -.2)
    assert result["longest_s"] == {"C4": 4.4, "D4": 4.4, "E4": 4.4} and result["joined"] == "legato"


def test_partial_credit_in_order():
    assert play(frames((("C4",), 4.2)))[0]["score"] == .25
    assert play(frames((("C4",), 4.2), (("D4",), 4.2)))[0]["score"] == .5
    assert play(frames((("C4",), 4.2), ((), 2.), (("D4",), 4.2), ((), 2.), (("E4",), 4.2)))[0]["score"] == .75
    assert play(frames((("C4",), 3.9), (("D4",), 4.2), (("E4",), 4.2)))[0]["score"] == 0.      # Sa 0.1 s short
    assert play(frames((("D4",), 4.2), (("C4",), 4.2), (("E4",), 4.2)))[0]["score"] == .25     # Re before Sa


def test_one_hand_alone_scores_nothing():
    """Pumping with no key, or keys with no pumping, sound nothing; a full reservoir, then keys alone, gives Sa
    under 3 s (air.py: 2.96 s for one reed). None of these holds a note for 4 s."""
    assert play(frames(((), 20.)))[0]["score"] == 0.
    assert play(frames((("C4",), 2.96), ((), 1.5), (("D4",), .0), ((), 10.)))[0]["score"] == 0.


def test_full_credit_needs_unbroken_sound():
    assert play(phrase(0.))[0]["score"] == 1.                         # the next key sounds as the last one stops
    assert play(phrase(.1))[0]["score"] == 1.                         # two frames: the dropout a recording allows
    assert play(phrase(.15))[0]["score"] == .85                       # an audible break: detached
    assert play(phrase(.5))[0]["score"] == .85
    assert play(phrase(.6))[0]["score"] == .75                        # not joined at all
    assert play(phrase(-1.))[0]["score"] == 1.                        # legato overlap up to 1.0 s
    assert play(phrase(-1.1))[0]["score"] == .75                      # longer: a chord, not a phrase
    mixed = frames((("C4",), 4.4), (("D4",), 4.4), ((), .3), (("E4",), 4.4))
    assert play(mixed)[0]["score"] == .85


def test_short_dropouts_are_bridged_longer_ones_split_the_note():
    sa = frames((("C4",), 2.), ((), .1), (("C4",), 2.1))
    rest = frames((("D4",), 4.4), (("E4",), 4.4))
    assert play(sa + rest)[0]["score"] == 1.
    sa = frames((("C4",), 2.), ((), .15), (("C4",), 2.1))
    assert play(sa + rest)[0]["score"] == 0.


def test_other_keys_case_motion_and_release():
    stray = frames((("C4", "C#4"), .25))
    assert play(stray + PHRASE)[0]["score"] == .75                     # another key 0.25 s
    assert play(frames((("C4", "C#4"), .2)) + PHRASE)[0]["score"] == 1.
    moved = np.asarray(HOME) + (.012, 0., 0.)
    assert play(PHRASE, position=moved)[0]["score"] == .75             # 12 mm off its marks: not 1.0
    assert play(PHRASE, position=np.asarray(HOME) + (.031, 0., 0.))[0]["score"] == 0.
    tip = np.array(((1., 0., 0.), (0., np.cos(.1), -np.sin(.1)), (0., np.sin(.1), np.cos(.1))))
    assert play(PHRASE, rotation=tip)[0]["score"] == 0.               # tipped 5.7 degrees
    assert play(PHRASE, release_s=.9)[0]["score"] == .75               # not let go for a full second


def test_held_key_at_the_end_is_not_released():
    grade = PhraseGrade(start_xy=HOME[:2])
    result = None
    for f in PHRASE:
        result = grade.update(f, HOME, np.eye(3), True, False)
    for _ in range(40):
        result = grade.update((), HOME, np.eye(3), False, False)       # hands off, a key still down (stuck)
    assert result["score"] == .75 and not result["released"]


def test_void_is_latched():
    grade = PhraseGrade(start_xy=HOME[:2])
    grade.update((), np.asarray(HOME) + (.05, 0., 0.), np.eye(3), True, True)
    result = None
    for f in PHRASE:
        result = grade.update(f, HOME, np.eye(3), True, not f)
    for _ in range(30):
        result = grade.update((), HOME, np.eye(3), False, True)
    assert result["score"] == 0. and result["void"]
