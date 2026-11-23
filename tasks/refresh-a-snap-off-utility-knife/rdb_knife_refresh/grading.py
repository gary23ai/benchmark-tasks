"""Native scored fracture inside a desktop snapper and external containment and end-of-trial inventory."""
from dataclasses import dataclass
import numpy as np
from . import layout as L
@dataclass
class KnifeGrade:
 sample_hz:float=L.PHYSICS_HZ
 seated:bool=False
 broken:bool=False
 samples:int=0
 stable_samples:int=0
 void:str|None=None
 def update(self,poses,velocities,*,robot_counts,body_counts,body_forces,native_score_broken,snapper_contacts=()):
  self.samples+=1;p=np.asarray(poses,float);v=np.asarray(velocities,float);c=np.asarray(robot_counts);bc=np.asarray(body_counts);bf=np.asarray(body_forces,float)
  if p.shape!=(4,7) or v.shape!=(4,6) or c.shape!=(18,4) or bc.shape!=(4,4) or bf.shape!=(4,4,3) or not all(np.isfinite(x).all() for x in (p,v,c,bc,bf)) or np.any(c<0) or np.any(bc<0):
   self.void=self.void or 'invalid native state';return self.result(0.)
  if np.max(abs(np.linalg.norm(p[:,3:],axis=1)-1))>.001:self.void=self.void or 'nonunit native quaternion'
  R=L.quat_matrix(p[L.KNIFE,3:]);C=L.quat_matrix(p[L.CASE,3:]);B=L.quat_matrix(p[L.SEGMENT,3:]);A=L.quat_matrix(p[L.LEVER,3:])
  hinge=p[L.CASE,:3]+C@L.HINGE_LOCAL
  if np.linalg.norm(p[L.LEVER,:3]-hinge)>.001 or C[:,0]@A[:,0]<np.cos(np.deg2rad(1)):self.void=self.void or 'lever left its physical hinge'
  x0,x1,y0,y1=L.TABLE_BOUNDS
  if np.any(p[:,2]<-.01) or np.any(p[:,0]<x0) or np.any(p[:,0]>x1) or np.any(p[:,1]<y0) or np.any(p[:,1]>y1):self.void=self.void or 'a component left the workbench'
  if np.any(c[:,L.SEGMENT]):self.void=self.void or 'robot touched the exposed steel segment'
  knife_local=(p[L.KNIFE,:3]-p[L.CASE,:3])@C
  vertices=p[L.SEGMENT,:3]+L.segment_vertices()@B.T
  box_vertices=(vertices-p[L.CASE,:3])@C
  inside=bool(np.all(np.max(abs(box_vertices[:,:2]),axis=0)<L.INNER_HALF+.0005) and box_vertices[:,2].min()>L.INNER_FLOOR-.0005 and box_vertices[:,2].max()<L.INNER_TOP)
  # The visible insertion collar is level with the guide mouth when fully seated.
  collar_in_guide=bool(np.max(abs(knife_local[:2]))<=.004 and abs(knife_local[2]-L.SEAT_LOCAL[2])<=.003
                       and (C.T@R[:,0])[2]<-np.cos(np.deg2rad(8)))
  relative=C.T@A
  lever_angle=float(np.arctan2(relative[2,1],relative[1,1]))
  if collar_in_guide and lever_angle<=np.deg2rad(-2):
   self.seated=True
  if native_score_broken and not self.broken:
   if not self.seated or not collar_in_guide:
    self.void=self.void or 'segment separated outside the observed disposal operation'
   self.broken=True
  if not native_score_broken:
   expected=np.array((L.SCORE_X+L.SEGMENT_LENGTH/2,0,0))
   if np.linalg.norm((p[L.SEGMENT,:3]-p[L.KNIFE,:3])@R-expected)>.0005:
    self.void=self.void or 'unfractured segment detached'
  # Only an exterior breach fails the history test. Internal wall and floor
  # contacts are not conditions a physical referee has to reconstruct.
  outer_half=np.array((.031,.019))
  escaped=bool(np.any(np.abs(box_vertices[:,:2])>outer_half+.001) or
               box_vertices[:,2].min()<-.001 or box_vertices[:,2].max()>.117)
  if self.broken and escaped:self.void=self.void or 'fragment visible outside the collection chamber'
  # After withdrawal an end photograph can distinguish the shortened tip.
  # All four corners of the remaining oblique tip must clear the closed case
  # on at least one exterior face before the missing segment can be inspected.
  tip_corners=np.array([(L.SCORE_X+L.SCORE_SLOPE*y,y,z) for y in (-L.BLADE_WIDTH/2,L.BLADE_WIDTH/2)
                        for z in (-L.BLADE_THICKNESS/2,L.BLADE_THICKNESS/2)])
  tip_in_case=(p[L.KNIFE,:3]+tip_corners@R.T-p[L.CASE,:3])@C
  withdrawn=bool(tip_in_case[:,2].min()>.116 or np.any(tip_in_case[:,:2].min(axis=0)>outer_half)
                 or np.any(tip_in_case[:,:2].max(axis=0)<-outer_half))
  confirmed=bool(self.broken and withdrawn)
  # In hardware, a clean closed case gaining the weight of the missing
  # segment certifies inventory. It need not settle against a particular wall.
  recovered=bool(confirmed and inside)
  handle_ends=p[L.KNIFE,:3]+np.array(((L.HANDLE_END,0,0),(L.SCORE_X+.003,0,0)))@R.T
  onmat=bool(np.all(np.max(abs(handle_ends[:,:2]-L.MAT_CENTER[:2]),axis=0)<L.MAT_HALF-.010) and handle_ends[:,2].min()>.005 and handle_ends[:,2].max()<.012 and abs(R[2,0])<np.sin(np.deg2rad(3)))
  upright=bool(C[2,2]>np.cos(np.deg2rad(5)) and abs(p[L.CASE,2])<.0015 and np.linalg.norm(p[L.CASE,:2]-L.CASE_START[:2])<.025)
  free=bool(not np.any(c));quiet=bool(np.max(np.linalg.norm(v[[L.CASE,L.KNIFE,L.LEVER],:3],axis=1))<.010 and np.max(np.linalg.norm(v[[L.CASE,L.KNIFE,L.LEVER],3:],axis=1))<.25)
  qualified=bool(self.broken and recovered and onmat and upright and free and quiet and not self.void)
  self.stable_samples=self.stable_samples+1 if qualified else 0;stable=max(0,self.stable_samples-1)/self.sample_hz
  score=1. if stable>=1. else .6 if recovered else .3 if confirmed else 0.
  return self.result(score,seated=self.seated,actual_seated=collar_in_guide,native_score_broken=bool(native_score_broken),visible_lever_press=self.seated,withdrawn=withdrawn,contained=inside,recovered=recovered,onmat=onmat,case_upright=upright,released=free,quiet=quiet,stable_seconds=stable,knife_in_case_m=knife_local.tolist(),lever_angle_rad=lever_angle)
 def result(self,score,**kw):return dict(score=0. if self.void else float(score),finished=bool(self.void or score==1.),void=self.void,broken=self.broken,stable_samples=self.stable_samples,sample_hz=self.sample_hz,**kw)
@dataclass(frozen=True)
class KnifeScorer:
 name:str='knife_refresh_score'
 def __call__(self,record,target):
  from inspect_robots.scorer import Score
  if not record.steps:return Score(value=0.,explanation='No actual robot actions')
  g=record.steps[-1].result.info['knife_grade'];return Score(value=g['score'],explanation=str(g))
def make_scorer():return KnifeScorer()
