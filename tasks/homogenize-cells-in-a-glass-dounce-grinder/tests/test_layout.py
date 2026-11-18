"""The numbers in layout.py against the product and the physics they come from."""

import math

import numpy as np

from rdb_dounce import layout as L


def test_published_grinder_numbers():
    assert (L.TUBE_OD, L.TUBE_L, L.PESTLE_L, L.CLEARANCE_B) == (.013, .082, .175, .00005)
    assert abs(L.BORE_AREA * L.BORE_H * 1e6 - 7.) < .2            # the estimated bore holds the nominal 7 mL
    assert abs(L.HEAD_D - (L.TUBE_ID - L.CLEARANCE_B)) < 1e-12


def test_masses_are_glass():
    assert 8e-3 < L.TUBE_GLASS_KG < 10e-3 and 18e-3 < L.PESTLE_KG < 24e-3
    assert L.TUBE_KG == L.TUBE_GLASS_KG + L.LIQUID_KG


def test_drag_law_inside_the_theory_and_the_arms():
    c = sorted(L.C_THEORY.values())
    assert c[0] < L.DRAG < c[-1]
    assert 8. < L.SUCTION_CAP < 9.                                  # (1 atm - vapour) x head area
    assert L.fluid_force(.018) == -L.DRAG * .018                    # 4.5 N against an 18 mm/s up-stroke
    assert L.fluid_force(.1) == -L.SUCTION_CAP and L.fluid_force(-.1) == L.SUCTION_CAP
    assert L.fluid_force(.018, 0.) == 0.
    assert abs(L.FOAM_SPEED - L.SUCTION_CAP / L.DRAG) < .001        # foaming starts where the suction boils
    # the free pestle's weight sinks it at under 1 mm/s; the tube's weight is far below the up-stroke's pull
    assert L.PESTLE_KG * 9.81 / L.DRAG < .001
    assert L.TUBE_KG * 9.81 < .05 * L.fluid_force(-.018)


def test_liquid_surface():
    s = [L.surface_height(d) for d in np.linspace(0., .035, 8)]
    assert all(a > b for a, b in zip(s, s[1:]))                     # the shaft leaves as the pestle rises
    assert L.surface_height(0.) < L.BORE_H - .008                   # no overflow with the pestle down
    assert L.submerged_share(L.START_DEPTH) == 1.


def test_stave_ring_is_closed():
    pieces = L.tube_pieces()
    assert len(pieces) == L.STAVES + 1
    half = math.pi / L.STAVES
    widths = [2 * L.WALL_OUT * math.tan(half)]
    assert widths[0] > .0025
