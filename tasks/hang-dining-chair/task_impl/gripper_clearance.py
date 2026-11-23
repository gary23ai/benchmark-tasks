"""Minimum surface gap from the six actual YAM gripper links to a fixture actor."""

import numpy as np

from .arm_release import PlacedHull
from .clearance import Part, closest_distance
from .geometry import primitive_rotation


def aabb_gap(first, second):
    return np.linalg.norm(np.maximum(np.abs(first.center - second.center) - first.bounds - second.bounds, 0.))


class GripperClearance:
    def __init__(self, actor):
        self.parts = [(p['type'], np.asarray(p['offset']), primitive_rotation(p['euler_deg']),
                       np.asarray(p['size']) / 2, p.get('toe_m')) for p in actor['parts']]

    def distance(self, frame, table_origin, robot_links):
        origin, basis, _ = frame
        closest = .04
        for kind, offset, rotation, half, toe in self.parts:
            region = Part(kind, origin + table_origin + basis @ offset, basis @ rotation, half, toe)
            for link in robot_links:
                if aabb_gap(region, link) >= closest:
                    continue
                if link.shapes is None:
                    link.shapes = [PlacedHull(mesh, link.position, link.basis) for mesh in link.link.meshes]
                for shape in link.shapes:
                    if aabb_gap(region, shape) < closest:
                        closest = min(closest, closest_distance(region, shape))
                        if closest == 0.:
                            return closest
        return closest
