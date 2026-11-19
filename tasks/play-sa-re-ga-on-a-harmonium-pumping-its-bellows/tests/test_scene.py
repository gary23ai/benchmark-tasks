"""The generated scene file agrees with layout.py and fixed_scene.py, and the harmonium is one joint tree."""

import importlib.util
import math
import re
from pathlib import Path

import numpy as np

from rdb_harmonium import fixed_scene as fixture
from rdb_harmonium import layout as L

ROOT = Path(__file__).resolve().parents[1]
TEXT = fixture.USD.read_text()


def builder():
    spec = importlib.util.spec_from_file_location("build_assets", ROOT / "tools/build_assets.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsRevoluteJoint|PhysicsPrismaticJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert builder().usd() == TEXT


def test_one_articulation_tree_without_loops():
    bodies = re.findall(r'def Xform "([^"]+)" \(\n\s+prepend apiSchemas = \["PhysicsRigidBodyAPI"', TEXT)
    assert bodies == list(fixture.LINKS) and len(bodies) == 41
    assert TEXT.count("PhysicsArticulationRootAPI") == 1 and "PhysicsArticulationRootAPI" in block("Xform", "case")
    parents = re.findall(r"rel physics:body0 = </Fixture/Actors/([^>]+)>", TEXT)
    children = re.findall(r"rel physics:body1 = </Fixture/Actors/([^>]+)>", TEXT)
    assert set(parents) == {"case"} and sorted(children) == sorted(fixture.LINKS[1:])    # a star: no loops
    assert "PhysicsFixedJoint" not in TEXT and "enabledSelfCollisions = 0" in TEXT


def test_masses():
    total = sum(float(m) for m in re.findall(r"float physics:mass = ([0-9.e-]+)", TEXT))
    assert abs(total - L.MASS) < 1e-4
    assert f"float physics:mass = {L.FLAP_MASS:.6g}" in block("Xform", "flap")


def test_bellows_hinge_and_its_torque():
    joint = block("PhysicsRevoluteJoint", fixture.FLAP_JOINT)
    assert 'uniform token physics:axis = "Y"' in joint
    assert "float physics:lowerLimit = 0" in joint
    assert f"float physics:upperLimit = {math.degrees(L.FLAP_OPEN):.6g}" in joint
    assert f"float drive:angular:physics:maxForce = {L.FLAP_TORQUE_NM:.6g}" in joint
    assert "drive:angular:physics:stiffness = 0" in joint
    assert f"targetVelocity = {math.degrees(L.FLAP_OPEN_SPEED):.6g}" in joint
    # opening turns the top edge away from the robot (+x)
    closed, open_ = L.flap_corner(0.), L.flap_corner(L.FLAP_OPEN)
    assert open_[0] - closed[0] > .05 and np.isclose(closed[0], L.BACK_X + L.FLAP_T)


def test_keys_are_sprung_slides_with_a_stop():
    assert TEXT.count("def PhysicsPrismaticJoint") == L.KEYS == 39
    for name in ("C4", "C#4", "E4"):
        joint = block("PhysicsPrismaticJoint", f"key_{L.prim_name(name)}_slide")
        assert 'uniform token physics:axis = "Z"' in joint
        assert f"float physics:lowerLimit = {-L.KEY_TRAVEL:.6g}" in joint and "float physics:upperLimit = 0" in joint
        assert f"stiffness = {L.KEY_RATE:.6g}" in joint and f"targetPosition = {L.KEY_PRELOAD:.6g}" in joint
    assert L.KEY_RATE * L.KEY_PRELOAD < 1. < L.KEY_RATE * (L.KEY_PRELOAD + L.KEY_TRAVEL) < 2.   # about 0.7 to 1.4 N


def test_colliders_are_hidden_boxes_and_display_meshes_do_not_collide():
    for kind, name, apis in re.findall(r'def (Cube|Mesh) "([^"]+)" \(\n\s+prepend apiSchemas = \[([^\]]*)\]', TEXT):
        if kind == "Mesh":
            assert "PhysicsCollisionAPI" not in apis, name
        else:
            assert "PhysicsCollisionAPI" in apis, name
    assert TEXT.count('token visibility = "invisible"') == TEXT.count('def Cube "')
    assert "Cylinder" not in re.findall(r'def (\w+) "', TEXT)


def test_keyboard_layout_and_marked_keys():
    assert L.KEY_NAMES[0] == "C2" and L.KEY_NAMES[-1] == "D5"
    whites = [m for m in L.KEY_MIDI if L.white_index(m) is not None]
    assert len(whites) == 23 and len(L.KEY_MIDI) - len(whites) == 16
    ys = [L.key_box(m)[1] for m in whites]
    assert np.allclose(np.diff(ys), -L.WHITE_PITCH) and abs(ys[0] + ys[-1]) < 1e-9
    for name, y in (("C4", -.066), ("D4", -.088), ("E4", -.110)):
        assert np.isclose(fixture.key_top(name)[1], y) and '"dot"' in block("Xform", f"key_{name}")
    assert np.isclose(L.frequency("A4"), 440.) and np.isclose(L.frequency("C4"), 261.63, atol=.01)


def test_case_on_its_feet_and_marks():
    x, y, z = fixture.CASE_START
    assert f"double3 xformOp:translate = ({x:.6g}, {y:.6g}, {z + fixture.TOP:.6g})" in block("Xform", "case")
    assert len(re.findall(r'def Cube "foot_\d"', TEXT)) == 4
    body = block("Xform", "case")
    lows = [float(v) for v in re.findall(r"xformOp:translate = \([^,]+, [^,]+, ([0-9.e-]+)\)", body)]
    assert min(lows) > 0.
