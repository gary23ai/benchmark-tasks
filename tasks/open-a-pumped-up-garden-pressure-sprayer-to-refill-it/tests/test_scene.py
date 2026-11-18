"""The generated scene file matches the layout, and the layout holds together."""

import math
import re

import numpy as np
import pytest

pytest.importorskip("scipy")

import build_assets  # noqa: E402
from rdb_sprayer_relief import fixed_scene as fixture  # noqa: E402
from rdb_sprayer_relief import layout as L  # noqa: E402

TEXT = fixture.USD.read_text()


def test_scene_file_is_what_the_builder_writes():
    assert TEXT == build_assets.usd()


def test_bodies_joints_and_masses():
    bodies = re.findall(r'def Xform "(\w+)" \(\n\s+prepend apiSchemas = \["PhysicsRigidBodyAPI"', TEXT)
    assert tuple(bodies) == fixture.LINKS
    assert TEXT.count("PhysicsArticulationRootAPI") == 1
    assert re.findall(r'def Physics(?:Revolute|Prismatic)Joint "(\w+)"', TEXT) == ["head", "knob", "valve"]
    masses = [float(m) for m in re.findall(r"float physics:mass = ([\d.e-]+)", TEXT)]
    assert np.allclose(masses, fixture.LINK_MASSES)
    assert abs(sum(masses) - (.35 + L.WATER_L)) < .01            # 0.35 kg empty (maker) plus the water
    assert f"float physics:upperLimit = {math.degrees(L.HEAD_FREE):.6g}" in TEXT       # 1.5 turns
    assert f"float drive:angular:physics:maxForce = {L.HEAD_DRAG:.6g}" in TEXT


def test_overall_size_is_the_makers():
    assert L.HEIGHT == .320 and 2 * L.TANK_R == pytest.approx(.134)
    assert L.KNOB_BOTTOM > L.HEAD_TOP > L.COLLAR_TOP > L.NECK_Z > L.BODY_TOP
    assert fixture.ORIGIN[2] == L.MAT[2] and L.TANK_R < L.MAT[0] / 2


def test_every_collider_is_rigid_convex_and_handled_parts_have_hidden_colliders():
    assert "physxDeformable" not in TEXT and "compliantContact" not in TEXT
    approximations = set(re.findall(r'physics:approximation = "(\w+)"', TEXT))
    assert approximations == {"convexHull"}
    for name in ("nozzle_collider", "cap_collider", "knob_collider", "wall_collider", "shoulder_collider"):
        block = TEXT[TEXT.index(f'def Mesh "{name}"'):]
        block = block[:block.index("}\n")]
        assert "PhysicsCollisionAPI" in block and 'visibility = "invisible"' in block
    for name in ("bottle", "collar", "nozzle", "cap", "knob", "neck_tolerance_tick"):      # drawn only
        block = TEXT[TEXT.index(f'def Mesh "{name}"'):]
        block = block[:block.index("}\n")]
        assert "PhysicsCollisionAPI" not in block and "invisible" not in block


def test_valve_spring_forces():
    stiffness = float(re.search(r"drive:linear:physics:stiffness = ([\d.e-]+)", TEXT).group(1))
    target = float(re.search(r"drive:linear:physics:targetPosition = ([\d.e-]+)", TEXT).group(1))
    assert np.isclose(stiffness * (0. - target), L.VALVE_SPRING[0])                   # shut
    assert np.isclose(stiffness * (L.VALVE_TRAVEL - target), L.VALVE_SPRING[1])       # fully out
    assert L.VALVE_OPEN < L.VALVE_TRAVEL


def test_valve_sits_square_on_the_shoulder():
    base = L.valve_base()
    assert np.isclose(base[2], L.shoulder_z(L.VALVE_BASE_R)) and L.NECK_R < L.VALVE_BASE_R < L.TANK_R
    slope = np.array((L.TANK_R - L.NECK_R, L.BODY_TOP - L.NECK_Z))                    # along the shoulder, (r, z)
    axis = L.valve_axis()
    assert abs(np.hypot(axis[0], axis[1]) * slope[0] + axis[2] * slope[1]) < 1e-9


def test_handle_passes_over_the_valve_when_the_head_turns():
    # highest point of the valve's round cap (pulled right out) against the handle's lowest edge
    # (parts of the one articulation do not collide with each other, so this is about the picture being right)
    cap_end = L.valve_cap_centre(L.VALVE_TRAVEL) + L.valve_axis() * L.VALVE_CAP_LEN / 2
    valve_top = cap_end[2] + L.VALVE_CAP / 2 * math.cos(L.SHOULDER_ELEV)
    assert L.HANDLE_BOTTOM - valve_top > .004
    assert L.NOZZLE_Z - L.NOZZLE_W / 2 > L.HANDLE_BOTTOM                              # and so does the nozzle


def test_no_gauge_and_unmodified_tank():
    assert "gauge" not in TEXT.lower() and "needle" not in TEXT.lower()
    assert not hasattr(L, "GAUGE_AZ") and not hasattr(L, "needle_angle")
    assert 'def Mesh "neck_tolerance_tick"' in TEXT                  # the hazard tolerance is drawn on the neck


def test_start_pose_and_instruction():
    assert np.degrees(L.NOZZLE_START) == pytest.approx(-5.) and tuple(fixture.MARK) == (.3, 0.)
    for word in ("pressure", "valve", "gauge", "vent", "release"):                    # the safety step is not given away
        assert word not in fixture.INSTRUCTION.lower()
