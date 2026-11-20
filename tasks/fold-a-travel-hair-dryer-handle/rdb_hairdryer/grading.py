"""Judge the real native hinge and released support, without controlling bodies."""
from dataclasses import dataclass,field
import numpy as np
from . import layout as L,fixed_scene as F
from .support_geometry import world_device_vertices
@dataclass
class DryerGrade:
 sample_hz:float=240.
 stable_samples:int=0
 hinge_angles:list[float]=field(default_factory=list)
 void:str|None=None
 def update(self,poses,velocities,*,body_contact_counts,robot_contact_counts,body_contact_forces):
  p=np.asarray(poses,float);v=np.asarray(velocities,float)
  if p.shape!=(L.N,7) or v.shape!=(L.N,6) or not np.isfinite(p).all() or not np.isfinite(v).all() or np.max(abs(np.linalg.norm(p[:,3:],axis=1)-1))>.02:
   self.void=self.void or 'invalid native body state';return self.result(0.)
  c=np.asarray(body_contact_counts);r=np.asarray(robot_contact_counts);f=np.asarray(body_contact_forces,float)
  if c.shape!=(L.N,L.N) or r.shape!=(18,L.N) or f.shape!=(L.N,L.N,3) or not np.isfinite(c).all() or not np.isfinite(r).all() or not np.isfinite(f).all() or np.any(c<0) or np.any(r<0):
   self.void=self.void or 'invalid native contact state';return self.result(0.)
  x0,x1,y0,y1=L.TABLE_BOUNDS
  if np.any(p[:,0]<x0) or np.any(p[:,0]>x1) or np.any(p[:,1]<y0) or np.any(p[:,1]>y1) or np.any(p[:,2]<-.010):self.void=self.void or 'a component left the counter'
  H=L.quat_matrix(p[L.HEAD,3:]);B=L.quat_matrix(p[L.HANDLE,3:]);rel=H.T@B
  anchor=p[L.HEAD,:3]+H@L.HINGE
  hinge_error=float(np.linalg.norm(p[L.HANDLE,:3]-anchor))
  angle=float(np.degrees(np.arctan2(rel[0,2],rel[0,0])))
  aligned=bool(rel[1,1]>np.cos(np.radians(3)))
  if hinge_error>.006 or not aligned:self.void=self.void or 'folding hinge separated'
  # Every loose-cord joint must remain attached. These are ordinary ball joints,
  # never proximity bonds. Reset authoring is the sole body-state write.
  ends=[]
  for i in range(6):
   C=L.quat_matrix(p[i+2,3:]);ends.append((p[i+2,:3]-C[:,2]*.025,p[i+2,:3]+C[:,2]*.025))
  cord_errors=[np.linalg.norm(ends[0][0]-(p[L.HANDLE,:3]+B@L.CABLE_ANCHOR_LOCAL))]
  cord_errors += [np.linalg.norm(ends[i][1]-ends[i+1][0]) for i in range(5)]
  cord_errors += [np.linalg.norm(ends[-1][1]-(p[L.BUNDLE,:3]+L.quat_matrix(p[L.BUNDLE,3:])@np.array((-.04,0,0))))]
  if max(cord_errors)>.008:self.void=self.void or 'cable detached'
  folded=bool(80<=angle<=93 and hinge_error<.002 and aligned)
  # All actual authored convex collision geometry participates in support.
  # Storage does not require an arbitrary barrel roll or head-root height.
  device,labels=world_device_vertices(p[:2],(H,B));lowest_index=int(np.argmin(device[:,2]));lowest=float(device[lowest_index,2])
  tx0,tx1,ty0,ty1=F.TABLE_SUPPORT_BOUNDS
  inside_table=bool(np.all((device[:,0]>=tx0)&(device[:,0]<=tx1)&(device[:,1]>=ty0)&(device[:,1]<=ty1)))
  supported=bool(-.001<lowest<.0012 and inside_table)
  free=bool(not r.any())
  quiet=bool(np.max(np.linalg.norm(v[:2,:3],axis=1))<.012 and np.max(np.linalg.norm(v[:2,3:],axis=1))<.15 and np.linalg.norm(v[L.BUNDLE,:3])<.025 and np.max(np.linalg.norm(v[2:8,:3],axis=1))<.050)
  basic=bool(folded and supported and free and quiet and not self.void)
  if basic:
   self.hinge_angles.append(angle)
   self.hinge_angles=self.hinge_angles[-(round(self.sample_hz)+1):]
  else:self.hinge_angles.clear()
  drift=float(max(self.hinge_angles)-min(self.hinge_angles)) if self.hinge_angles else 0.
  hinge_steady=bool(drift<=.1)
  qualified=bool(basic and hinge_steady)
  self.stable_samples=self.stable_samples+1 if qualified else 0
  if basic and not hinge_steady:self.hinge_angles=[angle]
  stable=max(0,self.stable_samples-1)/self.sample_hz
  score=1. if stable>=1 else .7 if folded else .4 if angle>=45 else 0.
  return self.result(score,angle_deg=angle,folded=folded,supported=supported,free=free,quiet=quiet,qualified=qualified,stable_seconds=stable,support_clearance_mm=lowest*1000,support_primitive=labels[lowest_index],device_inside_table=inside_table,hinge_error_mm=hinge_error*1000,max_cord_joint_error_mm=max(cord_errors)*1000,hinge_window_range_deg=drift,hinge_steady=hinge_steady)
 def result(self,score,**data):return dict(score=0. if self.void else float(score),finished=bool(self.void or score==1),void=self.void,**data)
@dataclass(frozen=True)
class DryerScorer:
 name:str='hairdryer_fold_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No completed action')
  g=record.steps[-1].result.info['dryer_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return DryerScorer()
