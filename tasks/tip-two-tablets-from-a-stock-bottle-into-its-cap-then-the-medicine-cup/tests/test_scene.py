"""The generated asset, the layout and the fixed scene agree."""

import re
import subprocess
import sys
from pathlib import Path

import numpy as np

from rdb_tablets_cap import fixed_scene as fixture
from rdb_tablets_cap import layout as L

ROOT = Path(__file__).resolve().parents[1]
USD = fixture.USD.read_text()


def test_asset_is_what_the_generator_writes():
    sys.path.insert(0, str(ROOT / "tools"))
    import build_assets

    assert build_assets.usd() == USD


def test_bodies_and_masses():
    names = re.findall(r'def Xform "(\w+)" \(\n            prepend apiSchemas = \["PhysicsRigidBodyAPI"', USD)
    assert tuple(names) == fixture.BODIES and len(names) == 3 + L.N_TABLETS
    masses = [float(v) for v in re.findall(r"float physics:mass = ([\d.e-]+)", USD)]
    assert np.allclose(masses, fixture.MASSES)
    assert "PhysicsJoint" not in USD and "kinematicEnabled" not in USD          # free rigid bodies only


def test_every_collider_is_convex_and_display_meshes_do_not_collide():
    assert USD.count('physics:approximation = "convexHull"') == USD.count('"PhysicsCollisionAPI"')
    assert USD.count('"PhysicsCollisionAPI"') == 3 * L.SIDES + 1 + L.SIDES + 1 + L.SIDES + 1 + L.N_TABLETS
    for block in re.findall(r'def Mesh "(?:shell|label\w*|ribs|mark_\d)" \((.*?)\)', USD, flags=re.S):
        assert "PhysicsCollisionAPI" not in block


def test_tablets_start_inside_the_bottle_without_touching():
    points = np.array([L.tablet_start(i) for i in range(L.N_TABLETS)])
    assert (np.hypot(points[:, 0], points[:, 1]) + L.TABLET_R < L.BODY_R_IN * np.cos(np.pi / L.SIDES)).all()
    assert points[:, 2].min() - L.TABLET_T / 2 > L.BOTTLE_FLOOR and points[:, 2].max() < L.BODY_H
    for i in range(L.N_TABLETS):
        for j in range(i):
            d = points[i] - points[j]
            assert np.hypot(d[0], d[1]) >= 2 * L.TABLET_R + .001 or abs(d[2]) >= L.TABLET_T + .0005


def test_things_fit_the_gripper_and_each_other():
    assert 2 * L.BOTTLE_R < .076 and 2 * L.CAP_R < .076
    assert L.CAP_H >= .015                                        # enough skirt for the finger pads
    assert 2 * L.NECK_R_IN > 2.5 * 2 * L.TABLET_R                 # tablets cannot bridge the mouth
    assert 2 * L.CAP_R_IN > 3 * 2 * L.TABLET_R and L.CAP_DEPTH > 2 * L.TABLET_T
    assert L.cup_inner_radius(L.CUP_H) > 2 * 2 * L.TABLET_R
    gaps = [np.hypot(*(np.subtract(a, b))) for a, b in ((fixture.BOTTLE_XY, fixture.CAP_XY), (fixture.CAP_XY, fixture.CUP_XY))]
    assert min(gaps) > .085
