"""Boxes that stand for the arms and grippers when the cutting wire is kept out of them. NumPy only.

Each YAM link collides in the simulator as a set of convex pieces. The wire model keeps its nodes out of the
bounding box of every such piece (in the link's own frame), so the wire cannot pass through a finger, the
gripper body or an arm link. The contact is one way: the wire is turned aside, the arm does not feel it.
"""

from functools import lru_cache
import struct
import xml.etree.ElementTree as ET

import numpy as np


def _vertices(path):
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + count * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rows["v"].reshape(-1, 3).astype(float)


@lru_cache(maxsize=1)
def pieces():
    """{link: (centres (k, 3), half extents (k, 3), centre and half extents of the box round them all)}."""
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    found = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        low, high = [], []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            origin = collision.find("origin")
            xyz = np.array((origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split(), float)
            rpy = np.array((origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split(), float)
            points = _vertices(YAM_URDF.parent / mesh.get("filename")) @ rpy_matrix(rpy).T + xyz
            low.append(points.min(axis=0))
            high.append(points.max(axis=0))
        if low:
            low, high = np.array(low), np.array(high)
            found[link.get("name")] = ((low + high) / 2, (high - low) / 2,
                                       (low.min(axis=0) + high.max(axis=0)) / 2,
                                       (high.max(axis=0) - low.min(axis=0)) / 2)
    return found


def boxes_near(link_poses, points, reach=.02):
    """World boxes (centre, rotation, half extents) of the pieces of every link that has a point within reach.

    ``link_poses`` is a list of {link name: (position, 3x3 rotation)}, one dict per arm, in the frame of the points.
    """
    out = []
    shapes = pieces()
    for arm in link_poses:
        for name, (position, rotation) in arm.items():
            if name not in shapes:
                continue
            centres, halves, centre, half = shapes[name]
            local = (points - position) @ rotation
            if not np.any(np.all(np.abs(local - centre) < half + reach, axis=1)):
                continue
            close = np.any(np.all(np.abs(local[:, None, :] - centres[None]) < halves[None] + reach, axis=2), axis=0)
            out.extend((position + rotation @ c, rotation, h) for c, h in zip(centres[close], halves[close]))
    return out


def fk_link_poses(joints, jaw, base):
    """Link poses of one arm from its six joint angles and its gripper opening (0 closed .. 1 open)."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import ALL_JOINTS, FINGER_OPEN
    from yam_arena.urdf import forward_kinematics

    q = dict(zip(ALL_JOINTS[:6], joints, strict=True))
    q["joint7"] = q["joint8"] = float(jaw) * FINGER_OPEN
    return {name: (pose[:3, 3] + np.asarray(base), pose[:3, :3])
            for name, pose in forward_kinematics(YAM_URDF, q).items()}
