"""The real trial's calibrated pitch rule agrees with the simulator's sounding keys."""

import numpy as np
import pytest

from rdb_harmonium import audio
from rdb_harmonium import layout as L
from rdb_harmonium.grading import PhraseGrade

CAL = audio.calibrate(audio.calibration_recording(), audio.RATE)


def frames(*parts, p=200.):
    out = []
    for names, seconds in parts:
        out += [(tuple(names), p if names else 0.)] * int(round(seconds * 20))
    return out


def judged_grade(timeline):
    judged = audio.judge(audio.synthesize(timeline), audio.RATE, *CAL)
    grade = PhraseGrade()
    result = None
    for f in judged:
        result = grade.update(f, (0., 0., 0.), np.eye(3), False, True)
    return judged, result


def test_calibration_covers_every_key():
    ref, noise, cross = CAL
    assert set(ref) == set(audio.CALIBRATION_KEYS) == set(L.KEY_NAMES) and len(ref) == 39
    assert all(ref[k] - noise[k] > 40. for k in ref)
    assert set(cross) == set(L.KEY_NAMES)
    seconds = len(audio.calibration_recording()) / audio.RATE
    assert 75. < seconds < 90.                                 # about 81 s at the bench


def sounding_keys(timeline):
    judged, _ = judged_grade(timeline)
    return {k for f in judged for k in f}, judged


def test_the_phrase_never_shows_a_stray_key():
    """The marked notes' overtones fall in other keys' bands (C4 on C5, D4 on D5); they are not stray keys."""
    keys, _ = sounding_keys(frames((("C4",), 2.), (("C4", "D4"), .3), (("D4",), 2.), (("E4",), 2.)))
    assert keys == {"C4", "D4", "E4"}


@pytest.mark.parametrize("stray", ["C2", "G#2", "C3", "F#3", "B3", "C5", "D5"])
def test_a_stray_key_anywhere_on_the_keyboard_is_heard_and_never_as_a_marked_note(stray):
    """(At a low note's first and last frame the long window also catches its neighbour semitone; both are
    unmarked, so the time counted against the stray-key limit is the same.)"""
    keys, judged = sounding_keys(frames(((stray,), 1.)))
    assert stray in keys and not keys & set(L.PHRASE)        # a low key's overtones are not Sa, Re or Ga
    assert sum(stray in f for f in judged) >= 18                # 18 or more of its 20 frames
    assert sum(any(k != stray for k in f) for f in judged[2:-2]) == 0


def test_a_stray_octave_played_with_a_marked_note_is_still_heard():
    _, judged = sounding_keys(frames((("C4", "C5"), 1.)))
    assert sum("C5" in f for f in judged) >= 15
    _, judged = sounding_keys(frames((("D4", "D5"), 1.)))
    assert sum("D5" in f for f in judged) >= 15


def test_older_eleven_key_calibrations_are_still_read():
    legacy = np.concatenate([np.concatenate((audio.tone(L.frequency(k), audio.LEGACY_NOTE_S, .2),
                                             np.zeros(int(audio.CAL_GAP_S * audio.RATE)))) for k in audio.LEGACY_KEYS]
                            + [np.zeros(int(audio.CAL_SILENCE_S * audio.RATE))])
    legacy = legacy + 1e-4 * np.random.default_rng(3).standard_normal(len(legacy))
    ref, noise, cross = audio.calibrate(legacy, audio.RATE)
    assert set(ref) == set(audio.LEGACY_KEYS) and set(cross) == set(audio.LEGACY_KEYS)


def test_the_phrase_is_read_back_from_its_sound():
    timeline = frames((("C4",), 4.2), (("C4", "D4"), .2), (("D4",), 4.0), (("D4", "E4"), .2), (("E4",), 4.2))
    judged, result = judged_grade(timeline)
    assert result["played"] == 1. and result["joined"] == "legato"
    for name in ("C4", "D4", "E4"):
        assert abs(result["longest_s"][name] - 4.4) <= .15
    differ = sum(a != tuple(sorted(s)) for a, (s, _) in zip(judged, timeline, strict=True))
    assert differ <= 8                                        # only at the note edges (0.256 s windows)


def test_a_break_between_notes_is_heard_to_within_one_frame():
    """The pitch windows (0.256 s) blur a 0.15 s silence away; the 50 ms reed-band check restores it."""
    for gap, kind in ((.15, "detached"), (.3, "detached"), (.05, "legato")):
        timeline = frames((("C4",), 4.4), ((), gap), (("D4",), 4.4), ((), gap), (("E4",), 4.4))
        _, result = judged_grade(timeline)
        assert result["joined"] == kind, (gap, result["joins_s"])
        assert all(abs(j - gap) <= .051 for j in result["joins_s"])


def test_neighbouring_keys_are_told_apart():
    judged, _ = judged_grade(frames((("C#4",), 1.)))
    assert {k for f in judged for k in f} == {"C#4"}
    judged, _ = judged_grade(frames((("C4", "D4"), 1.)))
    assert {k for f in judged for k in f} == {"C4", "D4"}


def test_quieter_pressure_still_counts_but_silence_and_knocks_do_not():
    judged, _ = judged_grade(frames((("D4",), 2.), p=160.))
    assert sum("D4" in f for f in judged) >= 38
    knock = np.zeros(audio.RATE * 2)
    knock[audio.RATE // 2:audio.RATE // 2 + 40] = .9          # a 2.5 ms click: broadband, no reed tone
    judged = audio.judge(knock, audio.RATE, *CAL)
    assert sum(bool(f) for f in judged) <= 2                  # at most one or two frames: never 0.2 s, never held
