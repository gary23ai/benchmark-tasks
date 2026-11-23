"""Check a real, flat-ended hex-key envelope against the moving fixture's collision shapes."""
import itertools
import math

import numpy as np


TOLERANCE_M=1e-6


def primitive_rotation(angles):
    x,y,z=np.radians(angles)
    cx,cy,cz=np.cos([x,y,z])
    sx,sy,sz=np.sin([x,y,z])
    # USD xformOp:rotateXYZ first applies X, then Y and Z to each local point.
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],
                     [sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],
                     [-sy,cy*sx,cy*cx]])


def key_vertices(spec):
    start,end=np.asarray(spec["pa"],float),np.asarray(spec["pb"],float)
    axis=end-start
    axis/=np.linalg.norm(axis)
    normal=np.array(spec["flat_axis"],float)
    normal-=axis*float(axis@normal)
    normal/=np.linalg.norm(normal)
    tangent=np.cross(axis,normal)
    radius=spec["across_flats_m"]/math.sqrt(3)
    angles=np.radians(30+60*np.arange(6))
    rim=radius*(np.cos(angles)[:,None]*normal+np.sin(angles)[:,None]*tangent)
    return np.vstack((start+rim,end+rim))


def box_intersects(vertices,half):
    """Separating-axis test for a convex regular hexagonal prism and a box."""
    longitudinal=vertices[6]-vertices[0]
    edges=np.vstack((np.diff(vertices[:4],axis=0),longitudinal))
    axes=np.vstack((np.eye(3),longitudinal,np.cross(longitudinal,edges[:3]),
                    np.cross(edges[:,None,:],np.eye(3)[None,:,:]).reshape(-1,3)))
    lengths=np.linalg.norm(axes,axis=1)
    axes=axes[lengths>1e-12]/lengths[lengths>1e-12,None]
    projected=vertices@axes.T
    extents=np.abs(axes)@half
    return not np.any((projected.min(axis=0)>extents+TOLERANCE_M)|
                      (projected.max(axis=0)<-extents-TOLERANCE_M))


def round_support(kind,half,direction):
    norm=np.linalg.norm(direction)
    if kind in {"capsule","sphere"}:
        point=half[0]*direction/norm
        if kind=="capsule":
            point[2]+=np.sign(direction[2])*(half[2]-half[0])
        return point
    radius_direction=direction[:2]
    unit=radius_direction/max(np.linalg.norm(radius_direction),1e-20)
    return np.array([*(half[0]*unit),np.sign(direction[2])*half[2]])


def closest_simplex(points):
    """Project the origin onto the Minkowski simplex and retain the active face."""
    best=None
    best_square=math.inf
    for size in range(1,min(4,len(points))+1):
        for face in itertools.combinations(points,size):
            face=np.array(face)
            if size==1:
                weights=np.array([1.])
            else:
                span=(face[1:]-face[0]).T
                coefficients=np.linalg.lstsq(span,-face[0],rcond=None)[0]
                weights=np.r_[1-coefficients.sum(),coefficients]
                if weights.min() < -1e-10:continue
            projected=weights@face
            square=float(projected@projected)
            if square<best_square:
                best_square=square
                best=(list(face[weights>1e-10]),projected)
    return best


def round_intersects(vertices,kind,half):
    """GJK intersection with exact support points for spheres, cylinders and capsules."""
    def support(d):
        return vertices[int(np.argmax(vertices@d))]-round_support(kind,half,-d)
    points=[support(np.array([1.,0.,0.]))]
    closest=points[0]
    for _ in range(40):
        distance=np.linalg.norm(closest)
        if distance<=TOLERANCE_M:return True
        direction=-closest/distance
        new=support(direction)
        if new@direction < -TOLERANCE_M:return False
        if any(np.linalg.norm(new-prev)<1e-12 for prev in points):return True
        points,closest=closest_simplex(points+[new])
    # An unresolved tangency cannot establish clear access.
    return True


class HexGauge:
    """One finite service gauge fixed to its socket and one real obstruction actor."""
    def __init__(self,spec,obstruction):
        self.socket=spec["a"]
        self.obstruction=spec["b"]
        self.local=key_vertices(spec)
        self.parts=[(p["type"],np.array(p["offset"]),primitive_rotation(p["euler_deg"]),np.array(p["size"])/2)
                    for p in obstruction["parts"]]

    def blocked(self,frames):
        origin,rotation=frames[self.socket][:2]
        other,other_rotation=frames[self.obstruction][:2]
        body_vertices=(self.local@rotation.T+origin-other)@other_rotation
        for shape,offset,orient,half in self.parts:
            vertices=(body_vertices-offset)@orient
            if np.any((vertices.min(axis=0)>half+TOLERANCE_M)|(vertices.max(axis=0)<-half-TOLERANCE_M)):
                continue
            if shape=="box":
                intersects=box_intersects(vertices,half)
            else:
                intersects=round_intersects(vertices,shape,half)
            if intersects:return 1.0
        return 0.0
