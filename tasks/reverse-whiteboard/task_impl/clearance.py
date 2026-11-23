"""Distance between the finite collision solids of two rigid fixture actors."""
import numpy as np

from .geometry import closest_simplex, primitive_rotation, round_support


TOLERANCE_M=1e-7


class Part:
    def __init__(self, kind, center, basis, half, toe=None):
        self.kind=kind
        self.center=np.asarray(center)
        self.basis=np.asarray(basis)
        self.half=np.asarray(half)
        self.bounds=np.abs(self.basis)@self.half
        self.vertices=None
        if kind=='wedge':
            x,y,z=self.half
            self.vertices=np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],
                                    [-x,-y,z],[x,-y,z],[x,y,-z+toe],[-x,y,-z+toe]])

    def support(self, direction):
        local=self.basis.T@direction
        if self.vertices is not None:
            point=self.vertices[int(np.argmax(self.vertices@local))]
        elif self.kind=='box':
            point=np.sign(local)*self.half
        else:
            point=round_support(self.kind,self.half,local)
        return self.center+self.basis@point


def closest_distance(first, second):
    """Return a conservative separation using converged support-plane bounds."""
    def support(direction):
        return first.support(direction)-second.support(-direction)

    direction=second.center-first.center
    if np.linalg.norm(direction)<TOLERANCE_M:direction=np.array([1.,0.,0.])
    active=[support(direction)]
    closest=active[0]
    lower=0.
    for _ in range(48):
        upper=np.linalg.norm(closest)
        if upper<=TOLERANCE_M:return 0.
        normal=closest/upper
        point=support(-normal)
        lower=max(lower,float(normal@point))
        if upper-lower<=TOLERANCE_M:return lower
        if any(np.linalg.norm(point-old)<1e-11 for old in active):return lower
        active,closest=closest_simplex([*active,point])
    return lower


class SolidClearance:
    def __init__(self, one, two):
        self.names=(one['name'],two['name'])
        self.local=[
            [(part['type'],np.asarray(part['offset']),primitive_rotation(part['euler_deg']),
              np.asarray(part['size'])/2,part.get('toe_m')) for part in actor['parts']] for actor in (one,two)
        ]
        if any(part[0] not in {'box','cylinder','sphere','capsule','wedge'} for group in self.local for part in group):
            raise ValueError('Unsupported collision solid in clearance measurement')

    def distance(self, frames):
        groups=[]
        for name,parts in zip(self.names,self.local,strict=True):
            center,basis,_=frames[name]
            groups.append([Part(kind,center+basis@offset,basis@orient,half,toe)
                           for kind,offset,orient,half,toe in parts])
        nearest=float('inf')
        for left in groups[0]:
            for right in groups[1]:
                gap=np.maximum(np.abs(left.center-right.center)-left.bounds-right.bounds,0.)
                if np.linalg.norm(gap)>=nearest:continue
                nearest=min(nearest,closest_distance(left,right))
                if nearest==0:return 0.
        return nearest
