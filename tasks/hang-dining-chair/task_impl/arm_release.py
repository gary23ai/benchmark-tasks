"""Check release volumes against YAM collision geometry at the measured link poses."""

from itertools import product

import numpy as np

from .clearance import Part, TOLERANCE_M, closest_distance
from .geometry import primitive_rotation


class Hull:
    def __init__(self, points):
        points = np.unique(points, axis=0)
        if len(points) < 4 or not np.isfinite(points).all():
            raise RuntimeError('Invalid robot collision mesh')
        low, high = points.min(axis=0), points.max(axis=0)
        self.center = (low + high) / 2
        self.half = (high - low) / 2
        self.points = points - self.center


class PlacedHull:
    def __init__(self, hull, position, basis):
        self.center = position + basis @ hull.center
        self.bounds = np.abs(basis) @ hull.half
        self.points = hull.points
        self.basis = basis

    def support(self, direction):
        point = self.points[np.argmax(self.points @ (self.basis.T @ direction))]
        return self.center + self.basis @ point


class LinkGeometry:
    def __init__(self, view, meshes):
        self.view = view
        self.meshes = [Hull(points) for points in meshes]
        all_points = np.concatenate(meshes)
        low, high = all_points.min(axis=0), all_points.max(axis=0)
        self.center = (low + high) / 2
        self.half = (high - low) / 2

    def place(self):
        from yam_arena.urdf import quat_xyzw_to_matrix

        tensor = self.view.get_transforms()
        tensor = tensor.torch if hasattr(tensor, 'torch') else tensor
        pose = tensor.detach().cpu().numpy()[0]
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise RuntimeError('Missing current YAM collision link pose')
        return PlacedLink(self, pose[:3], quat_xyzw_to_matrix(pose[3:]))


class PlacedLink:
    def __init__(self, link, position, basis):
        self.center = position + basis @ link.center
        self.bounds = np.abs(basis) @ link.half
        self.shapes = None
        self.link = link
        self.position = position
        self.basis = basis

    def collides(self, box):
        if not overlap_bounds(box, self):
            return False
        if self.shapes is None:
            self.shapes = [PlacedHull(hull, self.position, self.basis) for hull in self.link.meshes]
        return any(overlap_bounds(box, mesh) and closest_distance(box, mesh) <= TOLERANCE_M
                   for mesh in self.shapes)


def overlap_bounds(a, b):
    return np.all(np.abs(a.center - b.center) <= a.bounds + b.bounds + TOLERANCE_M)


def robot_colliders(stage, physics, environment_path, asset_links):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    output = []
    for arm in ('LeftArm', 'RightArm'):
        for relative in asset_links.values():
            path = environment_path + '/' + arm + relative
            link = stage.GetPrimAtPath(path)
            if not link or not link.HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError('Missing authoritative YAM rigid link ' + path)
            meshes = []
            to_link = cache.GetLocalToWorldTransform(link).GetInverse()
            subtree = iter(Usd.PrimRange(link, Usd.TraverseInstanceProxies()))
            for prim in subtree:
                if prim != link and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    subtree.PruneChildren()
                    continue
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                    continue
                if prim.IsA(UsdGeom.Mesh):
                    points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                elif prim.IsA(UsdGeom.Cube):
                    half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
                    points = list(product((-half, half), repeat=3))
                else:
                    raise RuntimeError('Unrecognized native robot collision shape ' + str(prim.GetPath()))
                relative_pose = cache.GetLocalToWorldTransform(prim) * to_link
                meshes.append(np.array([tuple(relative_pose.Transform(Gf.Vec3d(*p))) for p in points]))
            if meshes:
                view = physics.create_rigid_body_view(path)
                if list(view.prim_paths) != [path]:
                    raise RuntimeError('Physics view did not bind the measured link ' + path)
                output.append(LinkGeometry(view, meshes))
    if len(output) != 2 * len(set(asset_links.values())):
        raise RuntimeError('At least one native YAM link has no bound collision mesh')
    return output


class ReleaseWitness:
    def __init__(self, actor, margin):
        self.parts = [(np.array(part['offset']), primitive_rotation(part['euler_deg']),
                       np.array(part['size']) / 2 + margin) for part in actor['parts']]

    def clear(self, frame, table_origin, robot_links):
        position, basis, _ = frame
        for offset, rotation, half in self.parts:
            box = Part('box', position + table_origin + basis @ offset, basis @ rotation, half)
            if any(link.collides(box) for link in robot_links):
                return 0.
        return 1.
