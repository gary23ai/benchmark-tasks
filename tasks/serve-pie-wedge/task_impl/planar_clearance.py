"""Measure clearance of actual fixture solids after projection onto the bench."""

import numpy as np

from .clearance import Part, SolidClearance, closest_distance


class ProjectedPart:
    def __init__(self, part):
        self.part = part
        self.center = part.center.copy()
        self.center[2] = 0.
        self.bounds = part.bounds.copy()
        self.bounds[2] = 0.

    def support(self, direction):
        point = self.part.support(direction)
        point[2] = 0.
        return point


class PlanarClearance(SolidClearance):
    def distance(self, frames):
        groups = []
        for name, parts in zip(self.names, self.local, strict=True):
            center, basis, _ = frames[name]
            groups.append([ProjectedPart(Part(kind, center + basis @ offset, basis @ orient, half, toe))
                           for kind, offset, orient, half, toe in parts])
        nearest = float('inf')
        for left in groups[0]:
            for right in groups[1]:
                gap = np.maximum(np.abs(left.center - right.center) - left.bounds - right.bounds, 0.)
                if np.linalg.norm(gap) >= nearest:
                    continue
                nearest = min(nearest, closest_distance(left, right))
                if nearest == 0.:
                    return 0.
        return nearest
