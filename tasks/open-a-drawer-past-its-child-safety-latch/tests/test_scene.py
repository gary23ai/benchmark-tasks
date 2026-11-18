"""The generated scene file agrees with layout.py, and the layout with the published sizes."""

import importlib.util
import re
from pathlib import Path

import numpy as np

from rdb_child_latch import fixed_scene as fixture
from rdb_child_latch import layout as L

ROOT = Path(__file__).resolve().parents[1]
TEXT = fixture.USD.read_text()
INCH = .0254


def builder():
    spec = importlib.util.spec_from_file_location("build_scene", ROOT / "tools/build_scene.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def block(kind, name):
    start = TEXT.index(f'def {kind} "{name}"')
    following = re.search(r'\n        def (Xform|PhysicsPrismaticJoint|PhysicsRevoluteJoint|Material) "', TEXT[start + 10:])
    return TEXT[start:start + 10 + following.start()] if following else TEXT[start:]


def test_scene_file_is_what_the_generator_writes():
    assert builder().usd() == TEXT


def test_published_sizes():
    assert np.allclose(L.FRAME_SIZE, (17 * INCH, 14.375 * INCH, 6.875 * INCH), atol=.0006)
    assert np.allclose(L.DRAWER_INNER, (16.375 * INCH, 11.875 * INCH, 4.875 * INCH), atol=.0006)
    assert np.allclose(L.LATCH_PUBLISHED, (2.5 * INCH, 1. * INCH, 1.5 * INCH))
    assert abs(L.CATCH_OPEN - 1.5 * INCH) < 1e-9 and L.LATCH_Y == 0.
    # the catch ends up about 1 in behind the frame's front edge, as the sister model's guide places it
    assert abs(L.CATCH_FACE_X - L.FRAME_FRONT_X - 1. * INCH) < .003


def test_frame_drawer_and_latch_fit():
    b = builder()
    frame = {name: (np.array(low), np.array(high)) for name, low, high in b.frame_boxes()}
    low = np.min([v[0] for v in frame.values()], axis=0)
    high = np.max([v[1] for v in frame.values()], axis=0)
    assert np.allclose(high - low, L.FRAME_SIZE) and low[0] == L.FRAME_FRONT_X and low[2] == 0.
    drawer = {name: (np.array(low), np.array(high)) for name, low, high in b.drawer_boxes()}
    inside = (drawer["wall_back"][0][0] - drawer["front"][1][0], drawer["wall_left"][0][1] - drawer["wall_right"][1][1],
              drawer["wall_left"][1][2] - drawer["floor"][1][2])
    assert np.allclose(inside, L.DRAWER_INNER)
    for name, (lo, hi) in drawer.items():
        if name == "pull":
            continue
        lo, hi = lo + (L.FRAME_FRONT_X, 0, 0), hi + (L.FRAME_FRONT_X, 0, 0)
        assert hi[2] <= L.RAIL_Z - .002 and lo[2] >= .010 + .002
        assert abs(lo[1]) <= L.FRAME_SIZE[1] / 2 - L.FRAME_WALL - .002
    # the latch: 2.5 in from the drawer front's inner face to the back of the hook, 1 in high, 1.5 in wide
    latch = {name: (np.array(low), np.array(high)) for name, low, high in b.latch_boxes()}
    assert abs(latch["hook"][1][0] + L.HINGE_X - L.FRONT_T - L.LATCH_PUBLISHED[0]) < 1e-9
    assert abs(L.PLATE_Z[1] - L.PLATE_Z[0] - L.LATCH_PUBLISHED[1]) < 1e-9
    assert abs(latch["arm"][1][1] - latch["arm"][0][1] - L.LATCH_PUBLISHED[2]) < 1e-9
    # at rest the hook stands 5 mm into the catch's height and clears the frame's top by 5 mm
    _, clo, chi = b.catch_box()
    assert abs(L.HOOK_TOP_Z - clo[2] - .005) < 1e-9 and chi[2] == L.RAIL_Z and L.RAIL_Z - L.HOOK_TOP_Z >= .004
    # shut, the arm passes under the catch with room to spare, and the hook is behind it
    assert L.ARM_TOP_Z < clo[2] - .003
    assert L.drawer_x(0.) + L.HOOK_FACE_X > chi[0]
    # the ramp's top corner is the hook's top; its back face is a 45-degree slope
    centre, half = b.ramp()
    assert abs(centre[2] + L.HINGE_Z + half[0] * np.sqrt(2.) - L.HOOK_TOP_Z) < 1e-9


def test_bodies_masses_and_start_poses():
    for name, mass in zip(L.ACTORS, L.MASSES, strict=True):
        body = block("Xform", name)
        assert f"float physics:mass = {mass:.6g}" in body
        assert "PhysicsRigidBodyAPI" in body and "PhysxContactReportAPI" in body
        x, y, z = fixture.WORLD_START[name]
        assert f"double3 xformOp:translate = ({x:.6g}, {y:.6g}, {z:.6g})" in body
    assert "PhysicsArticulationRootAPI" not in TEXT
    assert "</Fixture/Actors/drawer>" in block("Xform", "latch")       # the latch does not collide with its drawer


def test_every_drawn_box_collides():
    cubes = re.findall(r'def Cube "[^"]+" \(\n\s+prepend apiSchemas = \[([^\]]+)\]', TEXT)
    assert len(cubes) == 5 + 1 + 7 + 3 and all("PhysicsCollisionAPI" in c for c in cubes)
    assert "Mesh" not in TEXT and "Sphere" not in TEXT


def test_drawer_slide_is_a_friction_capped_slide_with_no_spring():
    joint = block("PhysicsPrismaticJoint", "drawer_slide")
    assert "drive:linear:physics:stiffness = 0\n" in joint
    assert f"drive:linear:physics:maxForce = {L.RUNNER_FRICTION_N:.6g}\n" in joint
    assert "physics:lowerLimit = 0\n" in joint and f"physics:upperLimit = {L.OPEN_LIMIT:.6g}\n" in joint
    assert "rel physics:body0" not in joint and "rel physics:body1 = </Fixture/Actors/drawer>" in joint


def test_latch_pin_is_a_preloaded_torsion_spring_in_degrees():
    joint = block("PhysicsRevoluteJoint", "latch_pin")
    deg = np.pi / 180.
    assert f"drive:angular:physics:stiffness = {L.LATCH_STIFFNESS * deg:.6g}\n" in joint
    assert f"drive:angular:physics:targetPosition = {-np.degrees(L.LATCH_PRELOAD):.6g}\n" in joint
    assert "physics:lowerLimit = 0\n" in joint and f"physics:upperLimit = {np.degrees(L.LATCH_TRAVEL):.6g}\n" in joint
    assert 'physics:axis = "Y"' in joint
    assert "rel physics:body0 = </Fixture/Actors/drawer>" in joint and "rel physics:body1 = </Fixture/Actors/latch>" in joint
    assert TEXT.count("def PhysicsRevoluteJoint") == 1 and TEXT.count("def PhysicsPrismaticJoint") == 1
    # nothing ties the latch to the frame or the catch: only its hook meeting the catch holds the drawer
    assert "Static/catch" not in TEXT.split('def Scope "Joints"')[1]


def test_positive_pin_angle_presses_the_hook_down():
    # a rotation about +y by a positive angle takes the arm's +x end down (the sign the scorer's diagnostics use)
    angle = np.radians(5.)
    turn = np.array(((np.cos(angle), 0., np.sin(angle)), (0., 1., 0.), (-np.sin(angle), 0., np.cos(angle))))
    assert (turn @ (1., 0., 0.))[2] < 0.
