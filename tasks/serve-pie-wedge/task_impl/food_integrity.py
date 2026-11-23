"""Witness points for the seven unserved sectors on the settled pie surface."""
import numpy as np

NEIGHBORS=tuple(f"slice{i}" for i in range(1,8))
FOOD=("crust","filling",*NEIGHBORS)
FORCE_NOISE_N=.000001
MAX_DRIFT_M=.024
MAX_TILT_DEG=15.0
EXTRA_LANE_M=.0075
PRESS_SECONDS=.25

def witness_points(fixture):
    actors={a["name"]:a for a in fixture["actors"]}
    origin=np.asarray(actors["tin"]["position"],dtype=float)
    marks=[]
    for name in NEIGHBORS:
        p=np.asarray(actors[name]["position"],dtype=float)
        theta=np.arctan2(p[1]-origin[1],p[0]-origin[0])
        # The top surface just inside the tip and both ends of the outside crust.
        offsets=[]
        for r,offset in ((.010,0),(.095,-20),(.095,20)):
            a=theta+np.deg2rad(offset)
            offsets.append([origin[0]+r*np.cos(a)-p[0],origin[1]+r*np.sin(a)-p[1],.0108])
        marks.append(offsets)
    return np.asarray(marks)

def world_marks(poses,points):
    """Rigid transforms as [x,y,z,qx,qy,qz,qw]; numerical quaternion noise is normalized."""
    poses=np.asarray(poses,dtype=float)
    q=poses[:,3:]; q=q/np.linalg.norm(q,axis=1)[:,None]
    xyz=q[:,:3,None].transpose(0,2,1);w=q[:,3,None,None]
    twice=2*np.cross(xyz,points)
    return poses[:,:3,None].transpose(0,2,1)+points+w*twice+np.cross(xyz,twice)

def drifts(poses,points,reference):
    return np.linalg.norm(world_marks(poses,points)-reference,axis=2).max(axis=1)

class FoodGuard:
    """Measure each free sector against its own settled initial pose."""
    def __init__(self,fixture):
        self.points=witness_points(fixture)
        actors={a["name"]:a for a in fixture["actors"]}
        self.weight=np.array([actors[n]["mass_kg"]*9.81 for n in NEIGHBORS])
        self.pressed=np.zeros(len(NEIGHBORS))
        # Momentum of one whole neighbor after a drop through its own thickness.
        # This is a benchmark impact scale, not the fracture threshold of pastry.
        self.impact_scale=self.weight/9.81*np.sqrt(2*9.81*.022)
        self.impulses=np.zeros((12,len(NEIGHBORS)))
        self.impact_index=0
        tin=np.array(actors["tin"]["position"][:2])
        self.lane_points=[]
        self.normals=[]
        for j,angle in ((0,202.5),(6,157.5)):
            theta=np.deg2rad(angle)
            radial=np.array([np.cos(theta),np.sin(theta)])
            normal=np.array([-radial[1],radial[0]])*(1 if j==0 else -1)
            self.normals.append(normal)
            center=np.array(actors[NEIGHBORS[j]]["position"][:2])
            self.lane_points.append([[*(tin+r*radial-center),.0108] for r in (.035,.080)])
        self.lane_points=np.asarray(self.lane_points)
        self.normals=np.asarray(self.normals)

    def reset(self,poses):
        self.reference=world_marks(poses,self.points)
        self.lane_reference=world_marks(poses[[0,6]],self.lane_points)
        self.pressed.fill(0.)
        self.impulses.fill(0.)
        self.impact_index=0

    def tick(self,poses,forces,dt):
        # Per food, filters are the left gripper, right gripper, and downward
        # loading by server or served portion. Direct touches do not need a hold.
        poses=np.asarray(poses)
        out={}
        for name,contact in zip(FOOD,forces):
            out["left_"+name]=float(contact[0])
            out["right_"+name]=float(contact[1])
            if len(contact)>3:
                out["touch_left_"+name]=float(contact[3])
                out["touch_right_"+name]=float(contact[4])
        drift=drifts(poses,self.points,self.reference)
        normalized=poses[:,3:]/np.linalg.norm(poses[:,3:],axis=1)[:,None]
        tilt=np.degrees(np.arccos(np.clip(1-2*(normalized[:,0]**2+normalized[:,1]**2),-1,1)))
        down=np.asarray(forces)[2:,2]
        self.pressed=np.where(down>=self.weight,self.pressed+dt,0.)
        self.impulses[self.impact_index]=np.maximum(0.,down-self.weight)*dt
        self.impact_index=(self.impact_index+1)%len(self.impulses)
        impulse_ratio=self.impulses.sum(axis=0)/self.impact_scale
        for i,name in enumerate(NEIGHBORS):
            out["shift_"+name]=float(drift[i])
            out["tilt_"+name]=float(tilt[i])
            out["press_"+name]=float(self.pressed[i])
            out["impact_"+name]=float(impulse_ratio[i])
        delta=world_marks(poses[[0,6]],self.lane_points)-self.lane_reference
        for i,j in enumerate((0,6)):
            # Both witness points on the same cut must move outward to open a lane.
            # Tilt and tangential or rigid pan motion do not directly count as opening.
            out["lane_"+NEIGHBORS[j]]=float((delta[i,:,:2]@self.normals[i]).min())
        return out

def gripper_contacts(force,separation,counts,starts):
    """Separate speculative positive-gap patches from touching or force-bearing points."""
    force=np.asarray(force).ravel()
    separation=np.asarray(separation).ravel()
    counts=np.asarray(counts).ravel()
    starts=np.asarray(starts).ravel()
    if counts.sum() >= len(force):
        raise RuntimeError("Food-contact patch buffer reached its capacity")
    touching=[]
    for index in range(6):
        begin=int(starts[index]);end=begin+int(counts[index])
        touching.append(bool(np.any((separation[begin:end]<=0.) |
                                    (np.abs(force[begin:end])>=FORCE_NOISE_N))))
    return any(touching[:3]),any(touching[3:])
