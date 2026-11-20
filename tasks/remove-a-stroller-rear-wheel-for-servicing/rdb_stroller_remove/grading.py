"""Native release, complete axle removal and stable released maintenance rack."""
from dataclasses import dataclass
import numpy as np
from . import layout as L
@dataclass
class WheelGrade:
 sample_hz:float=240.
 void:str|None=None
 release_seen:bool=False
 old_removed:bool=False
 stable_samples:int=0
 def update(self,poses,velocities,*,robot_counts):
  p=np.asarray(poses,float);v=np.asarray(velocities,float);c=np.asarray(robot_counts)
  if p.shape!=(3,7) or v.shape!=(3,6) or c.shape!=(18,3) or not all(np.isfinite(x).all() for x in (p,v,c)) or np.any(c<0):
   self.void=self.void or 'invalid native state';return self.result(0.)
  if np.max(abs(np.linalg.norm(p[:,3:],axis=1)-1))>.001:self.void=self.void or 'nonunit native orientation'
  if np.linalg.norm(p[L.PIN,:2]-L.PIN_CENTER[:2])>.0005 or L.quat_matrix(p[L.PIN,3:])[2,2]<np.cos(np.deg2rad(1)):
   self.void=self.void or 'retainer separated from its physical guide'
  if np.linalg.norm(p[L.OLD_AXLE,:3]-p[L.OLD_WHEEL,:3])>.0005:self.void=self.void or 'wheel bearing separated'
  travel=L.pin_travel(p)
  if not -.0085<=travel<=.0005:self.void=self.void or 'release slider left its guide'
  if travel<-.0018:self.release_seen=True
  axis=L.quat_matrix(p[L.OLD_WHEEL,3:])[:,0]
  vertical=L.RADIUS*np.sqrt(max(0.,1-axis[2]**2))+L.TIRE_WIDTH/2*abs(axis[2])
  if p[L.OLD_WHEEL,2]-vertical<-.006:self.void=self.void or 'wheel fell below workbench'
  if not(-.17<p[L.OLD_WHEEL,0]<.87 and -.50<p[L.OLD_WHEEL,1]<.50):self.void=self.void or 'wheel left work area'
  axle_axis=L.quat_matrix(p[L.OLD_AXLE,3:])[:,0]
  tip=p[L.OLD_AXLE,:3]+axle_axis*L.AXLE_END
  if p[L.OLD_AXLE,0]<L.CENTER[0]-.003 and not self.release_seen:self.void=self.void or 'old axle forced past unreleased retainer'
  if self.release_seen and tip[0]<L.BORE_FRONT-.008:self.old_removed=True
  stored=bool(self.old_removed and abs(p[L.OLD_WHEEL,0]-L.OLD_CRADLE[0])<.006 and abs(p[L.OLD_WHEEL,1]-L.OLD_CRADLE[1])<.012 and abs(p[L.OLD_WHEEL,2]-L.OLD_CRADLE[2])<.009 and axis[0]>.995)
  quiet=bool(np.linalg.norm(v[L.OLD_WHEEL,:3])<.010 and np.linalg.norm(v[L.OLD_AXLE,:3])<.010 and np.linalg.norm(v[L.OLD_WHEEL,3:])<.25)
  returned=bool(abs(travel)<.0003)
  free=bool(not np.any(c))
  qualified=bool(stored and quiet and returned and free and not self.void)
  self.stable_samples=self.stable_samples+1 if qualified else 0;stable=max(0,self.stable_samples-1)/self.sample_hz
  score=1. if stable>=1. else .75 if stored else .5 if self.old_removed else .25 if self.release_seen else 0.
  return self.result(score,pin_travel_m=travel,axle_tip_x_m=float(tip[0]),wheel_in_rack=stored,quiet=quiet,retainer_returned=returned,released=free,stable_seconds=stable)
 def result(self,score,**values):return dict(score=0. if self.void else float(score),finished=bool(self.void or score==1.),void=self.void,release_seen=self.release_seen,old_removed=self.old_removed,stable_samples=self.stable_samples,sample_hz=self.sample_hz,**values)
@dataclass(frozen=True)
class WheelScorer:
 name:str='stroller_remove_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No native task steps')
  g=record.steps[-1].result.info['wheel_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return WheelScorer()
