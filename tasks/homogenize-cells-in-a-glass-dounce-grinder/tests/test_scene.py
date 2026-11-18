"""The scene file against layout.py and fixed_scene.py (text checks, no USD library needed)."""

import re

import numpy as np

from rdb_dounce import fixed_scene as F
from rdb_dounce import layout as L

TEXT = F.USD.read_text()


def block(name, kind="Xform"):
    i = TEXT.index(f'def {kind} "{name}"')
    depth, j = 0, TEXT.index("{", i)
    for k in range(j, len(TEXT)):
        depth += {"{": 1, "}": -1}.get(TEXT[k], 0)
        if depth == 0:
            return TEXT[i:k + 1]
    raise AssertionError(name)


def test_two_loose_glass_bodies_with_their_masses():
    assert TEXT.count('"PhysicsRigidBodyAPI"') == 2
    for name, mass in (("tube", L.TUBE_KG), ("pestle", L.PESTLE_KG)):
        b = block(name)
        assert f"physics:mass = {mass:.6g}" in b


def test_fluid_joint_is_free_with_one_damping_drive():
    j = block(F.FLUID_JOINT, "PhysicsJoint")
    assert "PhysicsLimitAPI" not in j and "limit" not in j.lower()           # every axis free: it holds nothing
    assert f"drive:transX:physics:damping = {L.DRAG:.6g}" in j and "stiffness = 0" in j
    assert f"maxForce = {L.SUCTION_CAP:.6g}" in j
    assert "physics:collisionEnabled = 1" in j                               # the head still meets the bore
    assert "</Dounce/Actors/tube>" in j and "</Dounce/Actors/pestle>" in j


def test_glass_colliders_hidden_and_true_size():
    tube = block("tube")
    assert tube.count("PhysxConvexHullCollisionAPI") == L.STAVES + 1
    assert tube.count('visibility = "invisible"') == L.STAVES + 1
    pestle = block("pestle")
    assert f"double radius = {L.HEAD_R_SIM:.6g}" in pestle and f"double radius = {L.KNOB_R_SIM:.6g}" in pestle
    assert L.HEAD_R_SIM < L.WALL_IN                                          # the head fits the bore


def test_drawn_parts_have_no_collision():
    for name in ("tube_glass", "head_glass", "shaft_glass", "liquid", "ring_bottom", "ring_top", "ring_limit",
                 "seat_line", "block_top", "jack_platform"):
        assert "CollisionAPI" not in block(name, "Mesh")


def test_block_leaves_the_well_open():
    sizes = {}
    for name in ("block_front", "block_back", "block_left", "block_right"):
        b = block(name, "Cube")
        c = np.array([float(v) for v in re.search(r"translate = \(([^)]*)\)", b).group(1).split(",")])
        s = np.array([float(v) for v in re.search(r"scale = \(([^)]*)\)", b).group(1).split(",")])
        sizes[name] = (c, s)
    ax, ay = F.AXIS_XY
    front, back = sizes["block_front"], sizes["block_back"]
    assert abs((front[0][0] + front[1][0] / 2) - (ax - L.WELL_D / 2)) < 1e-6
    assert abs((back[0][0] - back[1][0] / 2) - (ax + L.WELL_D / 2)) < 1e-6
    left, right = sizes["block_left"], sizes["block_right"]
    assert abs((left[0][1] - left[1][1] / 2) - (ay + L.WELL_D / 2)) < 1e-6
    assert abs((right[0][1] + right[1][1] / 2) - (ay - L.WELL_D / 2)) < 1e-6
    base = block("block_base", "Cube")
    top = float(re.search(r"translate = \(([^)]*)\)", base).group(1).split(",")[2]) + \
        float(re.search(r"scale = \(([^)]*)\)", base).group(1).split(",")[2]) / 2
    assert abs(top - L.SEAT_Z) < 1e-6


def test_start_poses():
    assert np.allclose(F.TUBE_START, (*F.AXIS_XY, L.SEAT_Z))
    assert abs(F.PESTLE_START[2] - F.TUBE_START[2] - L.BOTTOM_T - L.START_DEPTH) < 1e-12
    assert any(np.allclose(w, F.AXIS_XY) for w in F.WELLS) and len(F.WELLS) == 20
