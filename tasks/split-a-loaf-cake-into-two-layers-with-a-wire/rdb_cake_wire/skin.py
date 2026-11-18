"""Render the cutting wire as a thin tube that follows the wire model's nodes."""

import numpy as np

SIDES = 6


class WireSkin:
    def __init__(self, stage, path, nodes, radius, color=(.78, .79, .80)):
        from pxr import Gf, UsdGeom, Vt

        mesh = UsdGeom.Mesh.Define(stage, path)
        self.points = mesh.CreatePointsAttr()
        self.extent = mesh.CreateExtentAttr()
        self.vectors = lambda data: Vt.Vec3fArray([Gf.Vec3f(*map(float, point)) for point in data])
        self.radius = radius
        angles = np.linspace(0., 2. * np.pi, SIDES, endpoint=False)
        self.ring = np.column_stack((np.cos(angles), np.sin(angles)))
        faces = [(r * SIDES + j, r * SIDES + (j + 1) % SIDES, (r + 1) * SIDES + (j + 1) % SIDES, (r + 1) * SIDES + j)
                 for r in range(nodes - 1) for j in range(SIDES)]
        mesh.CreateFaceVertexCountsAttr([4] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar("constant").Set(self.vectors([color]))

    def update(self, nodes):
        tangent = np.gradient(nodes, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1), 1e-9)[:, None]
        side = np.cross(tangent, (0., 0., 1.))
        flat = np.linalg.norm(side, axis=1) < 1e-3
        side[flat] = (1., 0., 0.)
        side /= np.linalg.norm(side, axis=1)[:, None]
        up = np.cross(side, tangent)
        verts = (nodes[:, None, :] + self.radius * (self.ring[None, :, :1] * side[:, None, :]
                                                    + self.ring[None, :, 1:] * up[:, None, :])).reshape(-1, 3)
        self.points.Set(self.vectors(verts))
        self.extent.Set(self.vectors((verts.min(axis=0), verts.max(axis=0))))
