"""Read-only full-corner debris containment and real tray reinsertion scoring."""
from dataclasses import dataclass
import numpy as np
from . import layout as L

@dataclass
class ToasterGrade:
 sample_hz:float=240.
 void:str|None=None
 withdrawn:bool=False
 emptied:bool=False
 stable_samples:int=0
 def update(self,poses,velocities,*,robot_tray_contacts,robot_toaster_contacts,robot_crumb_contacts):
  p=np.asarray(poses,dtype=float);v=np.asarray(velocities,dtype=float)
  if p.shape!=(len(L.LINKS),7) or v.shape!=(len(L.LINKS),6) or not np.isfinite(p).all() or not np.isfinite(v).all():
   self.void='invalid native body state';return self.result(0.)
  if robot_crumb_contacts:self.void='a robot directly touched bread crumbs'
  rt=L.quat_matrix(p[L.TRAY,3:]);rb=L.quat_matrix(p[L.TOASTER,3:]);tray=p[L.TRAY,:3]
  if np.any(p[:,2]<-.010):self.void='a task object fell off the worktop'
  # The unplugged appliance is an ordinary free rigid body on rubber feet.
  # It may be steadied by hand; moving the housing to dump its contents is not
  # the requested removable-tray cleaning operation.
  housing_shift=float(np.linalg.norm(p[L.TOASTER,:3]-L.TOASTER_CENTER))
  if housing_shift>.020 or rb[2,2]<np.cos(np.deg2rad(10.)):
   self.void='the toaster was displaced or tipped instead of removing its tray'
  floor=p[L.TRAY,:3]+L.box_corners((L.TRAY_SIZE[0],L.TRAY_SIZE[1],L.TRAY_WALL))@rt.T
  floor_in_body=(floor-p[L.TOASTER,:3])@rb
  if float(floor_in_body[:,1].min())>L.TOASTER_FRONT+.005:self.withdrawn=True
  in_tray=[];in_bin=[];over_bin=[]
  for i,body in enumerate(L.CRUMBS):
   vertices=p[body,:3]+L.box_corners(L.CRUMB_SIZES[i])@L.quat_matrix(p[body,3:]).T
   local=(vertices-tray)@rt
   tray_ok=bool(np.max(np.abs(local[:,0]))<L.TRAY_SIZE[0]/2+.0003 and np.max(np.abs(local[:,1]))<L.TRAY_SIZE[1]/2+.0003
      and float(local[:,2].min())>=-.0003 and float(local[:,2].max())<=L.TRAY_WALL+.001)
   b=vertices[:,:2]-L.BIN_CENTER[:2]
   # A piece touching the inner wall is contained; do not shrink the
   # real opening. 0.1 mm matches the authored native contact offset.
   footprint=bool(np.all(np.max(np.abs(b),axis=0)<=L.BIN_INNER/2+L.CONTACT_OFFSET))
   # The trajectory has not escaped through the bin's actual outside
   # bottom (z=0). Transient contact penetration into its4mm floor
   # is not a spill; final containment still requires the inner top.
   over=footprint and float(vertices[:,2].min())>=-L.CONTACT_OFFSET
   caught=over and float(vertices[:,2].min())>=L.BIN_FLOOR-.0005 and float(vertices[:,2].max())<=L.BIN_RIM
   in_tray.append(tray_ok);in_bin.append(caught);over_bin.append(over)
   if not tray_ok and not over:self.void='a bread crumb spilled outside the tray and waste-bin opening'
  all_bin=all(in_bin)
  if self.withdrawn and all_bin:self.emptied=True
  relative=(tray-p[L.TOASTER,:3])@rb-np.array((0.,0.,L.TRAY_CENTER[2]))
  seated=bool(np.all(np.abs(relative)<=np.array((.0015,.0015,.001))) and float(rt[:,2]@rb[:,2])>np.cos(np.deg2rad(2.)) and float(rt[:,1]@rb[:,1])>np.cos(np.deg2rad(2.)))
  still=bool(np.linalg.norm(v[L.TRAY,:3])<.010 and np.linalg.norm(v[L.TRAY,3:])<.10 and
    np.all(np.linalg.norm(v[L.CRUMBS,:3],axis=1)<.020))
  free=not (robot_tray_contacts or robot_toaster_contacts or robot_crumb_contacts)
  if self.void is None and self.withdrawn and self.emptied and all_bin and seated and still and free:self.stable_samples+=1
  else:self.stable_samples=0
  seconds=max(0,self.stable_samples-1)/self.sample_hz
  score=0. if self.void else 1. if seconds>=1. else .6 if self.emptied and all_bin else .25 if self.withdrawn else 0.
  return self.result(score,tray_position_m=tray.tolist(),housing_shift_m=housing_shift,crumbs_in_tray=sum(in_tray),crumbs_in_bin=sum(in_bin),crumbs_over_bin=sum(over_bin),tray_seated=seated,released=free,stable_seconds=seconds)
 def result(self,score,**values):return dict(score=float(score),finished=self.void is not None or score==1.,void=self.void,withdrawn=self.withdrawn,emptied=self.emptied,stable_samples=self.stable_samples,sample_hz=self.sample_hz,**values)

@dataclass(frozen=True)
class ToasterScorer:
 name:str='toaster_tray_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No native task steps')
  g=record.steps[-1].result.info['toaster_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return ToasterScorer()
