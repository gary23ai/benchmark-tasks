"""The air rule: the board's travel fills the reservoir, a reed drains it, and nothing else makes a note."""

import numpy as np

from rdb_harmonium import layout as L
from rdb_harmonium.air import Air, pressure

DT = .05
DOWN = {"C4": .006}
UP = {}


def test_board_force_and_volume_numbers():
    assert np.isclose(L.FEEDER_M3_PER_RAD, L.FLAP_W * L.FLAP_H ** 2 / 2)
    assert 10. < L.TOP_EDGE_PUSH_N < 13.                     # about 12 N at the top edge, under the arm's ~15 N
    assert np.isclose(L.FLAP_TORQUE_NM, L.WORK_PA * L.FEEDER_M3_PER_RAD + L.SPRING_NM)


def test_a_full_reservoir_holds_one_reed_under_four_seconds():
    full = Air(volume=L.RESERVOIR_M3)
    assert 2.5 < full.seconds_left(1) < 3.2 < 4.0 - .5      # at least half a second short of a held note
    air, t = Air(volume=L.RESERVOIR_M3, flap=L.FLAP_OPEN), 0.
    while air.step(DT, L.FLAP_OPEN, DOWN)[1]:
        t += DT
    assert t < 3.2


def test_no_air_no_sound_and_no_key_no_sound():
    air = Air(flap=L.FLAP_OPEN)
    for _ in range(100):
        p, sounding = air.step(DT, L.FLAP_OPEN, DOWN)          # key down, nobody pumping
        assert p == 0. and sounding == ()
    for k in range(40):                                        # pumping, no key down
        theta = L.FLAP_OPEN - .15 * (1 - np.cos(np.pi * k / 10)) / 2
        _, sounding = air.step(DT, theta, UP)
        assert sounding == ()
    assert air.volume > 0.


def test_shutting_the_board_feeds_the_reservoir_and_opening_it_does_not():
    air = Air(flap=L.FLAP_OPEN)
    air.step(DT, L.FLAP_OPEN - .05, UP)
    assert np.isclose(air.volume, .05 * L.FEEDER_M3_PER_RAD - L.LEAK_M3S * DT)
    before = air.volume
    air.step(DT, L.FLAP_OPEN, UP)                              # reopening draws outside air in, not reservoir air
    assert air.volume <= before


def test_relief_valve_and_pressure_band():
    air = Air(flap=L.FLAP_OPEN)
    air.step(DT, 0., UP)                                       # a full stroke: 2.8 L into a 1.2 L reservoir
    assert air.volume <= L.RESERVOIR_M3 and air.vented_m3 > 1e-3
    assert pressure(0.) == 0. and pressure(L.RESERVOIR_M3) == L.FULL_PA
    assert abs(pressure(L.SAG_M3) - L.EMPTY_PA) < 2. and pressure(L.SAG_M3 / 2) < L.SPEAK_PA


def test_a_note_lasts_while_the_other_hand_keeps_pumping():
    air = Air(flap=L.FLAP_OPEN)
    theta, sounding_frames = L.FLAP_OPEN, 0
    for k in range(int(12 / DT)):
        phase = (k * DT) % 2.4                                 # a stroke every 2.4 s: 0.6 s in, 0.6 s back out
        if phase < .6:
            theta = L.FLAP_OPEN - .18 * phase / .6
        elif phase < 1.2:
            theta = L.FLAP_OPEN - .18 + .18 * (phase - .6) / .6
        _, sounding = air.step(DT, theta, DOWN if k * DT > .7 else UP)
        sounding_frames += bool(sounding)
    assert sounding_frames * DT > 11.
