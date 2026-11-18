"""The USD scene and the layout agree with each other and with the published equipment numbers."""

import math
import re

import numpy as np

from rdb_pool_bridge import fixed_scene as F
from rdb_pool_bridge import layout as L

USD = F.USD.read_text()


def test_published_numbers():
    assert abs(2 * L.BALL_R - .05715) < 1e-6 and .156 <= L.BALL_MASS <= .170          # WPA ball
    assert abs(L.NOSE_H / (2 * L.BALL_R) - .635) < 1e-9                                  # WPA cushion height
    assert 4 * L.IN <= L.RAIL_W <= 7.5 * L.IN                                            # WPA rail width
    assert abs(math.degrees(L.CORNER_CUT) - 142) < 1e-9 and abs(math.degrees(L.SIDE_CUT) - 104) < 1e-9
    assert 1 * L.IN <= L.CORNER_SHELF <= 2.25 * L.IN and 0 <= L.SIDE_SHELF <= .375 * L.IN
    bed_above_floor = L.BED_Z - L.FLOOR_Z
    assert 29.25 * L.IN <= bed_above_floor <= 31 * L.IN + 1e-6                            # WPA bed height
    assert L.cue_diameter(0.) <= .014 and abs(L.CUE_L - 58 * L.IN) < 1e-9 and L.CUE_MASS <= .709


def test_one_body_per_actor_and_their_masses():
    for name in F.ACTORS:
        assert len(re.findall(rf'def Xform "{name}"', USD)) == 1
    assert len(re.findall(r'"PhysicsRigidBodyAPI"', USD)) == len(F.ACTORS)
    masses = [float(m) for m in re.findall(r"float physics:mass = ([0-9.e-]+)", USD)]
    assert sorted(masses) == sorted([L.CUE_MASS] + [L.BALL_MASS] * 3)
    assert f"point3f physics:centerOfMass = ({-L.CUE_CM_S:.7g}, 0, 0)" in USD


def test_convex_hulls_fit_physx():
    for block in re.findall(r'uniform token physics:approximation = "convexHull".*?point3f\[\] points = \[(.*?)\]\n', USD, re.S):
        assert block.count("(") <= 64


def test_materials_give_the_published_contact_numbers():
    def friction(name):
        return float(re.search(rf'def Material "{name}".*?physics:dynamicFriction = ([0-9.]+)', USD, re.S).group(1))

    def restitution(name):
        return float(re.search(rf'def Material "{name}".*?physics:restitution = ([0-9.]+)', USD, re.S).group(1))
    assert abs((friction("ball_one") + friction("cloth")) / 2 - .2) < 1e-9               # ball-cloth sliding
    assert .03 <= friction("ball_one") <= .08 and .92 <= restitution("ball_one") <= .98  # ball-ball
    assert .6 <= min(restitution("cushion"), restitution("ball_one")) <= .9              # ball-rail
    assert .71 <= min(restitution("tip_leather"), restitution("ball_one")) <= .75        # leather tip
    assert abs((friction("tip_leather") + friction("ball_cue")) / 2 - .6) < 1e-9
    assert USD.count('restitutionCombineMode = "min"') >= 10


def test_the_shot_is_on_the_table():
    for n in L.BALLS:
        x, y, z = L.BALL_START[n]
        assert L.X_N + L.BALL_R < x < L.X_F - L.BALL_R and L.Y_R + L.BALL_R < y < L.Y_L - L.BALL_R
    assert np.linalg.norm(L.GHOST_XY - L.ONE_XY) == np.float64(np.linalg.norm(L.GHOST_XY - L.ONE_XY))
    assert abs(np.linalg.norm(L.GHOST_XY - L.ONE_XY) - 2 * L.BALL_R) < 1e-9
    # the 1-ball's path to the aim point clears both jaw points by more than a ball's radius
    pk = L.POCKETS["near_left"]
    for jaw in pk["jaws"]:
        w = L.OB_DIR_XY
        rel = jaw - L.ONE_XY
        assert abs(w[0] * rel[1] - w[1] * rel[0]) > L.BALL_R + .02
    assert L.in_pocket(np.r_[pk["hole"], L.BED_Z - .06]) == "near_left" and not L.off_table(np.r_[pk["hole"], L.BED_Z - .06])


def test_the_cue_rests_on_the_rail_clear_of_the_side_pocket():
    left = L.cue_point(L.CUE_REST_TIP, L.CUE_REST_AXIS, L.BRIDGE_S)
    right = L.cue_point(L.CUE_REST_TIP, L.CUE_REST_AXIS, L.GRIP_S)
    gap = L.POCKETS["near_side"]["jaws"]
    assert right[1] < gap[1][1] - .05 and left[1] > gap[0][1] + .05      # both hands pick beside the side pocket
    assert L.RAIL_CAP_X[0] < L.CUE_REST_X < L.RAIL_CAP_X[1]


def test_bridge_height_and_angle():
    assert abs(L.BRIDGE_AXIS_Z - .0964) < 5e-4 and abs(math.degrees(L.PITCH) - 7.) < 1e-9
    assert .25 < L.BRIDGE_S < .33 and L.cue_diameter(L.BRIDGE_S) == .013          # pro-taper part of the shaft
    assert L.GRIP_S < L.CUE_CM_S - .15                                              # grip in front of the balance point
