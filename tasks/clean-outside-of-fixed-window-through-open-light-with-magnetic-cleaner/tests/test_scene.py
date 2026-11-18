"""Check the written scene file and the magnet model against the stated setup, without a simulator."""

import sys
from pathlib import Path
import re
import unittest

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parent)]
from rdb_window_wall import build_scene, cleaner_model as m, drawn, magnet  # noqa: E402

USD = Path(build_scene.__file__).with_name("window.usda")
YAM_GRIPPER_STROKE = 0.094  # yam_arena.constants: pads 94 mm apart when open
YAM_GRIP_LIMIT_N = 50.  # yam_arena.constants.GRIPPER: the driver caps grip force at 50 N
YAM_PAD_TIP_PAST_TCP = 0.013
G = 9.81


def block(text, kind, name):
    start = text.index(f'def {kind} "{name}"')
    return text[start:text.index("\n        }\n", start)]


def follow(speed, distance, *, friction=m.PAD_FRICTION, dt=1 / 2400):
    """Point-mass model of the released outside half on the glass while the inside half is slid along x at
    ``speed``: magnet pull from the model, Coulomb friction on the pad, gravity. Returns the worst lag along
    the stroke, the worst sag and the smallest pull towards the glass."""
    pos, vel, drive = np.zeros(2), np.zeros(2), 0.
    worst_lag = worst_sag = 0.
    least_pull = 1e9
    for _ in range(int(distance / speed / dt)):
        drive += speed * dt
        dx, dz = pos[0] - drive, pos[1]
        f = m._pair_force((dx, m.PANE_T, dz), m.POLE_STRENGTH)
        pull, lateral = f[1], np.array((f[0], f[2] - m.MASS * G))
        least_pull = min(least_pull, pull)
        limit = friction * max(pull, 0.)
        if np.linalg.norm(vel) < 1e-4 and np.linalg.norm(lateral) <= limit:
            vel[:] = 0.
        else:
            slip = vel / max(np.linalg.norm(vel), 1e-4)
            vel += (lateral - limit * slip) / m.MASS * dt
        pos += vel * dt
        worst_lag, worst_sag = max(worst_lag, -dx), max(worst_sag, -dz)
    return worst_lag, worst_sag, least_pull


class SceneFileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = USD.read_text()

    def test_file_matches_the_generator(self):
        self.assertEqual(self.text, build_scene.text())

    def test_true_size_and_mass(self):
        self.assertAlmostEqual(m.PAD_BASE, .150)
        self.assertAlmostEqual(m.PAD_HEIGHT, .130)
        self.assertAlmostEqual(m.HALF_THICK, .045)
        self.assertAlmostEqual(m.BAR_Z[1], m.HALF_THICK)
        np.testing.assert_allclose(m.TRIANGLE.mean(axis=0), 0., atol=1e-12)  # the frame origin is the centroid
        self.assertAlmostEqual(sum(m.PART_MASS[k] * n for k, n in (("pad", 1), ("shell", 1), ("bar", 1), ("post", 2))), m.MASS)
        for half in m.HALVES:
            body = block(self.text, "Xform", half)
            masses = [float(v) for v in re.findall(r"physics:mass = ([0-9.e-]+)", body)]
            self.assertAlmostEqual(masses[0], m.MASS)
            self.assertAlmostEqual(sum(masses[1:]), m.MASS)
            self.assertEqual(body.count('physics:approximation = "convexHull"'), 2)
            self.assertIn("physxRigidBody:enableCCD = 1", body)

    def test_prism_points_are_the_stated_triangle(self):
        points = np.array(build_scene.prism_points(0., m.BODY_T))
        self.assertAlmostEqual(np.ptp(points[:, 0]), .150)
        self.assertAlmostEqual(np.ptp(points[:, 1]), .130)
        self.assertAlmostEqual(np.ptp(points[:, 2]), .022)

    def test_handle_fits_the_gripper(self):
        self.assertLess(m.BAR_SIZE[1], YAM_GRIPPER_STROKE - .040)  # 35 mm to spare on each side of the bar
        # Fingertips reach 13 mm past the gripper's centre point; they stop above the shell.
        self.assertGreater(m.GRIP_POINT[2] - YAM_PAD_TIP_PAST_TCP - m.BODY_T, .004)
        # The posts leave the middle of the bar free for 20 mm wide finger pads.
        self.assertGreater(m.POST_X - m.POST_SIZE[0] / 2 - .010, .015)

    def test_start_poses(self):
        for half in m.HALVES:
            body = block(self.text, "Xform", half)
            xyz = [float(v) for v in re.search(r"double3 xformOp:translate = \(([^)]*)\)", body).group(1).split(",")]
            world = m.to_world((*m.START_UN[half], m.START_SETTLE))
            np.testing.assert_allclose(xyz, (world[0], world[1], m.TABLE_TOP_WORLD_Z + m.START_SETTLE), atol=1e-8)
            wxyz = [float(v) for v in re.search(r"quatf xformOp:orient = \(([^)]*)\)", body).group(1).split(",")]
            np.testing.assert_allclose(wxyz, (m.START_QUAT[half][3], *m.START_QUAT[half][:3]), atol=1e-8)
            rot = m.PANE_ROT.T @ m.rotation(m.START_QUAT[half])  # in the pane frame
            np.testing.assert_allclose(rot[:, 2], (0., 0., 1.), atol=1e-12)  # pad down, handle up
            np.testing.assert_allclose(np.abs(rot[:, 0]), (1., 0., 0.), atol=1e-12)  # handle bar parallel to the wall
            # The inside half's apex points at the wall, the outside half's at the robot; turning either up
            # about its handle bar puts it the right way round for its own face of the glass, apex up.
            np.testing.assert_allclose(rot[:, 1], (0., -m.SIDE[half], 0.), atol=1e-12)
            a = -m.SIDE[half] * np.pi / 2
            turn = np.array(((1., 0., 0.), (0., np.cos(a), -np.sin(a)), (0., np.sin(a), np.cos(a))))
            np.testing.assert_allclose(turn @ rot, m.on_glass_rotation(half), atol=1e-12)
            # Both start indoors, every corner 30 mm or more from the wall and on the table.
            corners = np.array((*m.START_UN[half], 0.)) + np.c_[m.TRIANGLE, np.zeros(3)] @ rot.T
            self.assertGreater(corners[:, 1].min(), m.WALL_N[1] + .030)
            world = m.to_world(corners)
            self.assertTrue(np.all((world[:, 0] > -.19) & (np.abs(world[:, 1]) < .65)))
        self.assertTrue(m.PANE_X[0] < m.START_UN["inside"][0] < m.PANE_X[1])  # blue: in front of the fixed light
        # white: in front of the stile, where the right arm can lay its hand along the handle bar
        self.assertTrue(m.OPENING_U[1] - m.PAD_BASE / 2 < m.START_UN["outside"][0] < m.STILE_U[1] + m.PAD_BASE / 2)
        apart = np.linalg.norm(np.subtract(m.START_UN["inside"], m.START_UN["outside"]))
        self.assertGreaterEqual(apart, m.FIELD_RANGE)  # no pull between them where they lie

    def test_pane_frame(self):
        self.assertAlmostEqual(m.PANE_YAW_DEG, 90.)
        np.testing.assert_allclose(m.to_pane(m.to_world((.3, .2, .1))), (.3, .2, .1), atol=1e-12)
        # u is tabletop y; n points back at the robot; the middle of the glass is at x = 410 mm.
        np.testing.assert_allclose(m.to_world((.25, 0., .1)), (m.WALL_X + m.WALL_T - m.PANE_T / 2, .25, .1), atol=1e-12)
        np.testing.assert_allclose(m.to_world((0., .1, 0.)), (m.GLASS_MID_X - .1, 0., 0.), atol=1e-12)
        np.testing.assert_allclose(m.PANE_ROT, m.rotation(m.PANE_QUAT), atol=1e-12)
        for name in ("pane", "sill", "stile", "cell_0_00", "cell_5_29"):
            wxyz = [float(v) for v in re.search(r"quatf xformOp:orient = \(([^)]*)\)", block(self.text, "Cube", name)).group(1).split(",")]
            np.testing.assert_allclose(wxyz, (m.PANE_QUAT[3], *m.PANE_QUAT[:3]), atol=1e-8)
        cell = [float(v) for v in re.search(r"double3 xformOp:translate = \(([^)]*)\)", block(self.text, "Cube", "cell_0_00")).group(1).split(",")]
        np.testing.assert_allclose(m.to_pane((cell[0], cell[1], cell[2] - m.TABLE_TOP_WORLD_Z)),
                                   (m.CELL_U[0], -m.PANE_T / 2 - .0003, m.CELL_Z[0]), atol=1e-8)

    def box(self, name):
        text = block(self.text, "Cube", name)
        scale = np.array([float(v) for v in re.search(r"xformOp:scale = \(([^)]*)\)", text).group(1).split(",")])
        centre = np.array([float(v) for v in re.search(r"double3 xformOp:translate = \(([^)]*)\)", text).group(1).split(",")])
        centre = m.to_pane(centre - (0., 0., m.TABLE_TOP_WORLD_Z))
        return centre - scale, centre + scale, text  # the cubes are written in the pane frame's axes

    def test_wall_window_and_band(self):
        boxes = build_scene.wall_boxes()
        self.assertEqual(set(boxes), {"sill", "head", "pier_right", "stile", "pier_left", "pane",
                                      "foot_a_in", "foot_a_out", "foot_b_in", "foot_b_out"})
        for name, (u, n, z) in boxes.items():
            lo, hi, text = self.box(name)
            np.testing.assert_allclose(lo, (u[0], n[0], z[0]), atol=1e-8)
            np.testing.assert_allclose(hi, (u[1], n[1], z[1]), atol=1e-8)
            self.assertNotIn("PhysicsRigidBodyAPI", text)  # fixed
            self.assertIn("PhysicsCollisionAPI", text)
        # The glass: 550 x 320 x 4, its outdoor face flush with the wall's outdoor face.
        lo, hi, _ = self.box("pane")
        np.testing.assert_allclose(hi - lo, (.550, .004, .320), atol=1e-9)
        self.assertAlmostEqual(lo[1], m.WALL_N[0])
        self.assertAlmostEqual(m.WALL_N[1] - m.WALL_N[0], m.WALL_T)
        self.assertAlmostEqual(m.PANE_MASS, 2500 * .55 * .32 * .004, places=3)
        # The open light is a real hole: no fixed box covers any point of it, and the boxes round it meet.
        solid = [(np.array((u[0], n[0], z[0])), np.array((u[1], n[1], z[1]))) for u, n, z in boxes.values()]
        for u in np.linspace(m.OPENING_U[0] + .001, m.OPENING_U[1] - .001, 9):
            for z in np.linspace(m.OPENING_Z[0] + .001, m.OPENING_Z[1] - .001, 9):
                point = np.array((u, m.WALL_MID_N, z))
                self.assertFalse(any(np.all(point >= lo) and np.all(point <= hi) for lo, hi in solid))
        for point in ((m.OPENING_U[0] - .001, m.WALL_MID_N, .2), (m.OPENING_U[1] + .001, m.WALL_MID_N, .2),
                      (0., m.WALL_MID_N, m.OPENING_Z[0] - .001), (0., m.WALL_MID_N, m.OPENING_Z[1] + .001),
                      (m.WALL_U[0] + .001, m.WALL_MID_N, .2), (m.WALL_U[1] - .001, m.WALL_MID_N, m.WALL_H - .001)):
            point = np.array(point)
            self.assertTrue(any(np.all(point >= lo) and np.all(point <= hi) for lo, hi in solid), point)
        self.assertAlmostEqual(m.OPENING_U[1] - m.OPENING_U[0], .430)
        self.assertAlmostEqual(m.OPENING_Z[1] - m.OPENING_Z[0], .320)
        self.assertAlmostEqual(m.STILE_U[1] - m.STILE_U[0], .010)
        self.assertEqual(m.PANE_X[0], m.STILE_U[1])
        self.assertLessEqual(m.WALL_U[1], .65)  # on the 1300 mm table
        # The band: 180 squares of 5 mm, drawn only (no collider), 350 mm from the stile, all on the glass.
        self.assertEqual(len(re.findall(r'def Cube "cell_\d_\d\d"', self.text)), m.CELLS)
        self.assertEqual(m.CELLS, 180)
        self.assertNotIn("PhysicsCollisionAPI", block(self.text, "Cube", "cell_0_00"))
        self.assertAlmostEqual(m.BAND_U[1] - m.BAND_U[0], .150)
        self.assertAlmostEqual(m.BAND_Z[1] - m.BAND_Z[0], .030)
        self.assertAlmostEqual(m.BAND_U[0] - m.STILE_U[1], .350)
        self.assertLess(m.BAND_U[1], m.PANE_X[1])
        # With the pad's centre at the working height the band lies across the lower, wider part of the pad:
        # the pad is 100 mm wide or more at every height of the band.
        for z in m.BAND_Z:
            width = m.PAD_BASE * (2 * m.PAD_HEIGHT / 3 - (z - m.WORK_Z)) / m.PAD_HEIGHT
            self.assertGreaterEqual(width, .100 - 1e-9)
            self.assertLess(z - m.WORK_Z, 2 * m.PAD_HEIGHT / 3)
            self.assertGreater(z - m.WORK_Z, -m.PAD_HEIGHT / 3)
        self.assertGreater(m.WORK_Z - m.PAD_HEIGHT / 3, m.PANE_CLEAR_Z[0] + .05)
        self.assertLess(m.WORK_Z + 2 * m.PAD_HEIGHT / 3, m.PANE_CLEAR_Z[1] - .05)

    def test_drawn_details_have_no_physics(self):
        # Everything drawn for looks only is a plain rendered prim: no collider, no mass, no physics material.
        drawn_scope = self.text[self.text.index('def Scope "Drawn"'):self.text.index('def Scope "Marks"')]
        self.assertEqual(len(re.findall(r'def Cube "drawn_', drawn_scope)), len(drawn.wall_shapes()))
        for chunk in [drawn_scope] + [block(self.text, "Mesh", f"drawn_{part}") for part in ("pad", "body", "handle")]:
            for word in ("Physics", "Physx", "physics:", "material:binding:physics"):
                self.assertNotIn(word, chunk)
        for name in list(drawn.WALL_COLORS) + list(drawn.HALF_COLORS):
            material = self.text[self.text.index(f'def Material "{name}"'):]
            material = material[:material.index("\n        }\n")]
            self.assertNotIn("Physics", material)
        # Each half still has exactly the colliders and masses that were run; they are only hidden from the render.
        for half in m.HALVES:
            body = block(self.text, "Xform", half)
            self.assertEqual(body.count('def Mesh "drawn_'), 3)
            self.assertEqual(body.count("PhysicsCollisionAPI"), 5)  # pad, shell, bar, two posts
            self.assertEqual(body.count('token visibility = "invisible"'), 5)
            self.assertEqual(len(re.findall(r"physics:mass = ", body)), 6)  # the body and its five colliders
        # No collider of the wall or the glass is hidden.
        static = self.text[self.text.index('def Scope "Static"'):self.text.index('def Scope "Drawn"')]
        self.assertNotIn("invisible", static)

    def test_drawn_halves_show_what_is_held(self):
        body, pad, handle = (mesh()[0] for mesh in (drawn.body_mesh, drawn.pad_mesh, drawn.handle_mesh))
        # Body and pad lie within the colliders' triangle and thickness (only the corners are rounded off).
        for points in (body, pad):
            self.assertTrue(np.all(m.in_pad(points[:, :2] * (1 - 1e-9))))
        self.assertAlmostEqual(body[:, 2].max(), m.BODY_T)
        self.assertAlmostEqual(pad[:, 2].min(), 0.)
        self.assertLess(body[:, 2].min(), m.PAD_T)  # the body's rim comes down beside the pad: a border
        # The pad is a separate layer, smaller than the body all round.
        self.assertGreater(np.linalg.norm(body[:, :2], axis=1).max() - np.linalg.norm(pad[:, :2], axis=1).max(), .002)
        # The drawn grip section is the bar itself (24 x 10 mm, same place) wherever a gripper holds it, and the
        # drawn legs enclose the posts, so the fingers are seen on what they really grip.
        grip = handle[(np.abs(handle[:, 0]) <= drawn.HANDLE_LEG_X - drawn.HANDLE_FILLET - .002) & (handle[:, 2] > m.BAR_Z[0] - .010)]  # not the feet
        np.testing.assert_allclose((grip[:, 1].min(), grip[:, 1].max()), (-m.BAR_SIZE[1] / 2, m.BAR_SIZE[1] / 2), atol=1e-9)
        np.testing.assert_allclose((grip[:, 2].min(), grip[:, 2].max()), m.BAR_Z, atol=1e-9)
        self.assertGreater(drawn.HANDLE_LEG_X - drawn.HANDLE_FILLET, .025 + .010 - .010)  # right pads span x -35..-15
        legs = handle[(np.abs(handle[:, 0]) > drawn.HANDLE_LEG_X - .008) & (handle[:, 2] < m.BAR_Z[0])]
        post_x = (m.POST_X - m.POST_SIZE[0] / 2, m.POST_X + m.POST_SIZE[0] / 2)
        self.assertLessEqual(np.abs(legs[:, 0]).min(), post_x[0] + 1e-9)
        self.assertGreaterEqual(np.abs(legs[:, 0]).max(), post_x[1] - 1e-9)
        self.assertLessEqual(handle[:, 2].min(), m.BODY_T)  # the legs go into the body
        self.assertAlmostEqual(handle[:, 2].max(), m.BAR_Z[1])
        self.assertLess(np.abs(handle[:, 0]).max(), m.PAD_BASE / 2)

    def test_drawn_wall_keeps_out_of_the_way(self):
        shapes = drawn.wall_shapes()
        out_n, in_n = m.WALL_N
        for name, (u, n, z, _) in shapes.items():
            # Nothing drawn stands more than 1 mm into the open light, through the wall and its drawn depth.
            if n[0] < in_n and n[1] > out_n - drawn.WALL_DEPTH:
                into_u = min(u[1], m.OPENING_U[1]) - max(u[0], m.OPENING_U[0])
                into_z = min(z[1], m.OPENING_Z[1]) - max(z[0], m.OPENING_Z[0])
                if into_u > 0 and into_z > 0:
                    self.assertLessEqual(min(into_u, into_z), drawn.FRAME_PROUD + 1e-9, name)
            # Nothing drawn covers the marker band, on either face.
            if n[0] < m.FACE_Y["inside"] + .010 and n[1] > m.FACE_Y["outside"] - .010:
                over_u = min(u[1], m.BAND_U[1]) - max(u[0], m.BAND_U[0])
                over_z = min(z[1], m.BAND_Z[1]) - max(z[0], m.BAND_Z[0])
                self.assertFalse(over_u > 0 and over_z > 0, name)
            # Nothing drawn indoors stands further from the board than the window board.
            self.assertLessEqual(n[1], in_n + drawn.BOARD_PROJECTION + 1e-9, name)
            self.assertGreaterEqual(n[0], out_n - drawn.WALL_DEPTH - 1e-9, name)
            self.assertTrue(m.WALL_U[0] - 1e-9 <= u[0] and u[1] <= m.WALL_U[1] + 1e-9 and z[1] <= m.WALL_H + 1e-9, name)
        # Drawn outdoors, the wall's depth leaves the glass and the open light in a clear reveal, and the
        # mullion is drawn indoors only, so nothing drawn is in the way of the outside half's swing past the stile.
        self.assertGreaterEqual(shapes["mullion"][1][0], out_n)
        # The cleaner, drawn, clears the glazing beads at the far end of its furthest stroke (u 531 measured).
        reach = np.abs(drawn.body_mesh()[0][:, 0]).max()  # the drawn body's half-width along the glass
        self.assertLess(reach, m.PAD_BASE / 2 - .005)  # rounded corners
        self.assertLess(.531 + reach + .002, shapes["bead_in_end"][0][0])

    def test_glass_is_see_through_in_the_render(self):
        glass = self.text[self.text.index('def Material "glass"'):]
        self.assertIn("float inputs:opacity = 0.25", glass[:700])

    def test_pad_friction_governs_on_glass(self):
        pad = self.text[self.text.index('def Material "pad"'):]
        self.assertIn('frictionCombineMode = "min"', pad[:600])
        self.assertIn(f"staticFriction = {m.PAD_FRICTION:.9g}", pad[:600])
        self.assertLess(m.PAD_FRICTION, m.GLASS_FRICTION)


class MagnetModelTest(unittest.TestCase):
    def test_sized_from_the_rating(self):
        self.assertAlmostEqual(m.hold_force(m.RATED_GLASS)[0], m.HOLD_AT_RATED_N, places=6)
        self.assertAlmostEqual(m.hold_force()[0], 17.7, delta=.1)  # through this 4 mm pane
        self.assertAlmostEqual(m.hold_force(0.)[0], 32.4, delta=.1)  # pad to pad, no glass
        self.assertAlmostEqual(m.hold_force(.030)[0], 1.0, delta=.05)

    def test_matches_the_force_applied_in_the_simulator(self):
        x = .30
        # In the tabletop frame, as the simulator applies it.
        a = m.to_world((x, m.PANE_T / 2, m.WORK_Z))
        b = m.to_world((x + .006, -m.PANE_T / 2, m.WORK_Z - .002))
        ra, rb = m.PANE_ROT @ m.on_glass_rotation("inside"), m.PANE_ROT @ m.on_glass_rotation("outside")
        f1, t1, f2, t2 = magnet.interaction(a, ra, b, rb)
        np.testing.assert_allclose(f1, -f2, atol=1e-9)
        pull, back_x, back_z = m.hold_force(dx=.006, dz=-.002)
        in_pane = m.PANE_ROT.T @ f2
        self.assertAlmostEqual(in_pane[1], pull, places=6)  # towards the glass (+n for the outside half)
        self.assertAlmostEqual(-in_pane[0], back_x, places=6)
        self.assertGreater(in_pane[2], 0.)  # pulled back up
        # Newton's third law for torques as well: no net torque on the pair about any point.
        ca = a + ra @ m.COM
        cb = b + rb @ m.COM
        np.testing.assert_allclose(t1 + t2 + np.cross(ca, f1) + np.cross(cb, f2), 0., atol=1e-9)

    def test_pads_attract_and_a_pad_repels_a_back(self):
        a = np.array((.30, m.PANE_T / 2, m.WORK_Z))
        b = np.array((.30, -m.PANE_T / 2, m.WORK_Z))
        _, _, f2, _ = magnet.interaction(a, m.on_glass_rotation("inside"), b, m.on_glass_rotation("outside"))
        self.assertGreater(f2[1], 0.)
        # The outside half put on back to front: its back against the glass.
        flipped = m.on_glass_rotation("outside") @ np.diag((1., -1., -1.))
        _, _, f2, _ = magnet.interaction(a, m.on_glass_rotation("inside"), b - (0., m.BODY_T, 0.), flipped)
        self.assertLess(f2[1], 0.)

    def test_no_force_beyond_range_and_none_at_the_start(self):
        start = [np.array((*m.START[h][:2], 0.)) for h in m.HALVES]
        f1, _, f2, _ = magnet.interaction(start[0], m.rotation(m.START_QUAT["inside"]),
                                          start[1], m.rotation(m.START_QUAT["outside"]))
        self.assertLess(np.linalg.norm(f2), .01)

    def test_a_released_half_stays_up(self):
        self.assertGreater(m.PAD_FRICTION * m.hold_force()[0], 2. * m.MASS * G)
        self.assertGreater(m.PAD_FRICTION * m.hold_force(m.RATED_GLASS)[0], 1.2 * m.MASS * G)

    def test_within_the_arms(self):
        # Sliding the pair: friction of both pads, well inside what two 35 N fingers hold on the bar.
        drag = 2 * m.PAD_FRICTION * m.hold_force()[0]
        self.assertLess(drag, 10.)
        self.assertLess(drag, .5 * 2 * m.HANDLE_FRICTION * 35.)
        self.assertLess(35., YAM_GRIP_LIMIT_N)
        self.assertLess(m.hold_force(0.)[0], 2 * 40.)  # even pad to pad it is under the two fingers' 40 N each

    def test_outside_half_follows_a_stroke(self):
        lag, sag, pull = follow(.045, .350)
        self.assertLess(lag, .006)
        self.assertLess(sag, .004)
        self.assertGreater(pull, 10.)
        # ... and still does at twice the friction; at five times the speed it lags more but is not lost.
        lag, sag, pull = follow(.045, .350, friction=.50)
        self.assertLess(lag, .012)
        lag, sag, pull = follow(.225, .350)
        self.assertLess(lag, .012)

    def test_sliding_on_from_below_meets_no_snap(self):
        # Set on the glass 55 mm below the inside half, the outside half feels almost nothing; the pull then
        # grows smoothly as it is slid up, and at no height is it pushed off the glass by more than 1.5 N.
        self.assertLess(abs(m.hold_force(dz=-.055)[0]), .5)
        self.assertLess(abs(m.hold_force(dz=-.055)[2]), .2)
        pulls = [m.hold_force(dz=-dz)[0] for dz in np.arange(.055, -1e-9, -.001)]
        self.assertGreater(min(pulls), -1.5)
        self.assertAlmostEqual(pulls[-1], m.hold_force()[0])
        back = [m.hold_force(dz=-dz)[2] for dz in np.arange(.055, .001, -.001)]
        self.assertGreater(min(back), -.1)  # never pushed away from alignment by more than 0.1 N ...
        self.assertGreater(min(back[15:]), 0.)  # ... and drawn towards it from 40 mm in
        # The model is the same above and below: the magnets lie along the handle bar.
        np.testing.assert_allclose(m.hold_force(dz=-.03), m.hold_force(dz=.03), atol=1e-9)

    def test_one_magnet_off_is_a_weaker_catch(self):
        shifted = m.hold_force(dx=2 * m.MAGNET_X)[0]
        self.assertGreater(shifted, 5.)
        self.assertLess(shifted, .6 * m.hold_force()[0])


if __name__ == "__main__":
    unittest.main()
