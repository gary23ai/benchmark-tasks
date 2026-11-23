"""Counterexamples for measured-state scoring, not native trial evidence."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rdb_knife_refresh import layout as L
from rdb_knife_refresh.grading import KnifeGrade
class GradeTests(unittest.TestCase):
 def setUp(self):
  self.p=L.START_POSES.copy();self.v=np.zeros((4,6));self.c=np.zeros((18,4),int);self.bc=np.zeros((4,4),int);self.bf=np.zeros((4,4,3))
 def call(self,g,native=False,contacts=()):return g.update(self.p,self.v,robot_counts=self.c,body_counts=self.bc,body_forces=self.bf,native_score_broken=native,snapper_contacts=contacts)
 def seat(self):
  r=L.SEAT_ROTATION;q=Rotation.from_matrix(r).as_quat();self.p[L.KNIFE]=np.r_[L.CASE_START+L.SEAT_LOCAL,q]
  self.p[L.SEGMENT]=np.r_[self.p[L.KNIFE,:3]+r@np.array((L.SCORE_X+L.SEGMENT_LENGTH/2,0,0)),q]
 def fracture(self,g):
  self.seat();self.p[L.LEVER,3:]=Rotation.from_euler('x',-3,degrees=True).as_quat();self.call(g);point=self.p[L.KNIFE,:3]+L.SEAT_ROTATION@np.array((L.SCORE_X,0,0))+[0,0,-.004]
  self.bc[L.SEGMENT,L.LEVER]=1
  out=self.call(g,True,[np.r_[point,0,1,0,2.]])
  self.bc[:]=0;return out
 def finish(self):
  self.p[L.KNIFE]=L.START_POSES[L.KNIFE]
  self.p[L.SEGMENT]=np.r_[L.CASE_START+[.01,.006,.00419],0,0,0,1]
 def test_do_nothing_zero(self):
  g=KnifeGrade()
  for _ in range(1000):r=self.call(g)
  self.assertEqual(r['score'],0)
 def test_pose_alone_cannot_fracture(self):
  g=KnifeGrade();self.seat()
  for _ in range(1000):r=self.call(g)
  self.assertFalse(r['broken']);self.assertEqual(r['score'],0)
 def test_break_without_visible_press_void(self):
  g=KnifeGrade();self.seat();self.call(g);self.assertTrue(self.call(g,True)['void'])
 def test_contact_without_native_event_zero(self):
  g=KnifeGrade();self.seat();self.bc[L.SEGMENT,L.LEVER]=1
  self.assertEqual(self.call(g,False,[[.36,.14,.10,0,1,0,2]])['score'],0)
 def test_unseated_native_break_void(self):
  g=KnifeGrade();self.bc[L.SEGMENT,L.LEVER]=1
  self.assertTrue(self.call(g,True,[[.36,.14,.10,0,1,0,2]])['void'])
 def test_fragment_escape_latched(self):
  g=KnifeGrade();self.assertTrue(self.fracture(g)['broken']);self.finish();self.p[L.SEGMENT,1]+=.04
  self.assertTrue(self.call(g,True)['void']);self.finish();self.assertEqual(self.call(g,True)['score'],0)
 def test_blade_contact_latched(self):
  g=KnifeGrade();self.c[8,L.SEGMENT]=1;self.assertTrue(self.call(g)['void']);self.c[:]=0;self.assertEqual(self.call(g)['score'],0)
 def test_free_body_fragment_without_fracture_void(self):
  g=KnifeGrade();self.p[L.SEGMENT,0]+=.001;self.assertTrue(self.call(g)['void'])
 def test_segment_drop_does_not_prove_tool_withdrawal(self):
  g=KnifeGrade();self.fracture(g);self.p[L.SEGMENT,:3]=L.CASE_START+[0,0,.010]
  self.p[L.KNIFE,2]+=.004
  r=self.call(g,True);self.assertFalse(r['withdrawn']);self.assertEqual(r['score'],0.)
 def test_clock_first_sample_not_elapsed_time(self):
  g=KnifeGrade();self.fracture(g);self.finish()
  for _ in range(960):r=self.call(g,True)
  self.assertNotEqual(r['score'],1);self.assertEqual(self.call(g,True)['score'],1)
 def test_held_or_moving_cannot_finish(self):
  for mode in ['held','moving','mat','tilted']:
   self.setUp();g=KnifeGrade();self.fracture(g);self.finish()
   if mode=='held':self.c[8,L.KNIFE]=1
   if mode=='moving':self.v[L.KNIFE,0]=.05
   if mode=='mat':self.p[L.KNIFE,1]=-.3
   if mode=='tilted':self.p[L.CASE,3:]=Rotation.from_euler('x',10,degrees=True).as_quat()
   for _ in range(962):r=self.call(g,True)
   self.assertNotEqual(r['score'],1,mode)
 def test_invalid_measurements_rejected(self):
  for mode in ['negative_counts','nan','negative_body_counts']:
   self.setUp();g=KnifeGrade()
   if mode=='negative_counts':self.c[0,0]=-1
   elif mode=='nan':self.p[0,0]=np.nan
   else:self.bc[0,1]=-1
   self.assertEqual(self.call(g)['score'],0);self.assertTrue(g.void)
 def test_legitimate_side_wall_contact_allowed(self):
  g=KnifeGrade();self.fracture(g);self.finish()
  self.p[L.SEGMENT,0]=L.CASE_START[0]+L.INNER_HALF[0]-L.segment_vertices()[:,0].max()+.0003
  for _ in range(961):r=self.call(g,True)
  self.assertIsNone(r['void']);self.assertEqual(r['score'],1)
 def test_fragment_beyond_outer_wall_latched(self):
  g=KnifeGrade();self.fracture(g);self.finish()
  self.p[L.SEGMENT,0]=L.CASE_START[0]+.040
  self.assertTrue(self.call(g,True)['void']);self.finish();self.assertEqual(self.call(g,True)['score'],0)
 def test_internal_fragment_need_not_touch_floor(self):
  g=KnifeGrade();self.fracture(g);self.finish();self.p[L.SEGMENT,2]+=.018
  for _ in range(961):r=self.call(g,True)
  self.assertEqual(r['score'],1.)
 def test_returned_tool_without_verified_separation_scores_zero(self):
  g=KnifeGrade();self.finish()
  self.assertEqual(self.call(g)['score'],0.)
 def test_box_outline_breach_before_recovery_is_void(self):
  g=KnifeGrade();self.fracture(g);self.finish();self.p[L.SEGMENT,2]=.14
  self.assertTrue(self.call(g,True)['void'])
  self.finish();self.assertEqual(self.call(g,True)['score'],0.)
 def test_torn_hinge_invalid(self):
  g=KnifeGrade();self.p[L.LEVER,0]+=.002;self.assertTrue(self.call(g)['void'])
 def test_signed_native_contact_force_allowed(self):
  g=KnifeGrade();self.seat();self.p[L.LEVER,3:]=Rotation.from_euler('x',-3,degrees=True).as_quat();self.call(g);point=self.p[L.KNIFE,:3]+L.SEAT_ROTATION@np.array((L.SCORE_X,0,0))+[0,0,-.004]
  self.bc[L.SEGMENT,L.LEVER]=1
  r=self.call(g,True,[np.r_[point,0,-1,0,-2.]])
  self.assertTrue(r['broken']);self.assertIsNone(r['void'])
if __name__=='__main__':unittest.main()
