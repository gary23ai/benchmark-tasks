"""Draw the bellows' leather folds between the case's back wall and the bellows board, so that every rendered frame
shows the bellows opening and closing. Display only, no collision: the board itself is a real body.

A double bellows opens as a wedge from its bottom hinge: two triangular gussets at the sides and, across the top,
a strip folded once inward (the fold's depth shrinks as the board opens). Same technique as the drawn-line skin of
the Etch A Sketch package that rendered in Isaac: one USD mesh whose points are rewritten each control step.
"""

import numpy as np

from . import layout as L

LEAF = .034                       # each of the two leaves of the top fold, wall to fold line (m)
INSET = .004                      # the folds sit this far inside the board's edges


def fold_points(theta):
    """The bellows' outline in the instrument frame: (points, faces). Top strip: wall top, fold line, board top,
    at both side ends; gussets: hinge, wall top, fold line, board top at each side."""
    wall_top = np.array((L.BACK_X, L.CASE_H - INSET))
    board_top = L.flap_point(theta, 0., L.FLAP_H - INSET)
    hinge = np.array(L.HINGE)
    mid = (wall_top + board_top) / 2
    half = np.linalg.norm(board_top - wall_top) / 2
    depth = np.sqrt(max(LEAF ** 2 - half ** 2, 0.))
    toward_hinge = (hinge - mid) / np.linalg.norm(hinge - mid)
    fold = mid + depth * toward_hinge
    y = L.FLAP_W / 2 - INSET
    points = []
    for side in (y, -y):
        for x, z in (hinge, wall_top, fold, board_top):
            points.append((x, side, z))
    faces = [[1, 2, 6, 5], [2, 3, 7, 6],             # the top strip, two leaves
             [0, 1, 2, 3], [7, 6, 5, 4]]             # the two side gussets
    return np.array(points), faces


class BellowsSkin:
    def __init__(self, stage, path, color=(.06, .05, .05)):
        from pxr import Gf, UsdGeom, Vt

        mesh = UsdGeom.Mesh.Define(stage, path)
        self.points = mesh.CreatePointsAttr()
        self.extent = mesh.CreateExtentAttr()
        self.vectors = lambda data: Vt.Vec3fArray([Gf.Vec3f(*map(float, point)) for point in data])
        points, faces = fold_points(L.FLAP_OPEN)
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar("constant").Set(self.vectors([color]))
        hidden = [(0., 0., -5.)] * len(points)           # points from the start, so the renderer never skips it
        self.points.Set(self.vectors(hidden))
        self.extent.Set(self.vectors((hidden[0], hidden[0])))
        self.shown = None

    def update(self, theta, case_position, case_rotation, top):
        """``theta``: board angle (rad); the case's pose in the tabletop frame."""
        key = (round(float(theta), 4), tuple(np.round(case_position, 4)))
        if key == self.shown:
            return
        self.shown = key
        local, _ = fold_points(theta)
        verts = np.asarray(case_position) + (0., 0., top) + local @ np.asarray(case_rotation).T
        self.points.Set(self.vectors(verts))
        self.extent.Set(self.vectors((verts.min(axis=0), verts.max(axis=0))))
