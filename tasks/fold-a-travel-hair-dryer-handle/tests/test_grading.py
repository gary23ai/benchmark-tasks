"""Rubric edge cases; these constructed states are not simulation evidence."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rdb_hairdryer import layout as L
from rdb_hairdryer.grading import DryerGrade
from rdb_hairdryer.support_geometry import world_device_vertices


def state(angle=90.,roll=90.):
    p=L.START_POSES.copy();H=Rotation.from_euler('x',roll,degrees=True).as_matrix();B=H@Rotation.from_euler('y',angle,degrees=True).as_matrix()
    p[L.HEAD,3:]=Rotation.from_matrix(H).as_quat();p[L.HANDLE,:3]=p[L.HEAD,:3]+H@L.HINGE;p[L.HANDLE,3:]=Rotation.from_matrix(B).as_quat()
    vertices,_=world_device_vertices(p[:2],(H,B));p[:2,2]-=vertices[:,2].min()
    anchor=p[L.HANDLE,:3]+B@L.CABLE_ANCHOR_LOCAL;drop=(anchor[2]-.0125)/6;horizontal=np.sqrt(.05**2-drop**2);nodes=[anchor]
    for a in np.radians((90,45,-45,-90,0,0)):nodes.append(nodes[-1]+[horizontal*np.cos(a),horizontal*np.sin(a),-drop])
    for i in range(6):p[i+2,:3]=(nodes[i]+nodes[i+1])/2;p[i+2,3:]=L.align_z(nodes[i+1]-nodes[i])
    p[L.BUNDLE,:3]=nodes[-1]+[.04,0,0];p[L.BUNDLE,3:]=(0,0,0,1)
    return p,np.zeros((L.N,6)),dict(body_contact_counts=np.zeros((L.N,L.N)),robot_contact_counts=np.zeros((18,L.N)),body_contact_forces=np.zeros((L.N,L.N,3)))

def hold(g,p,v,c,n=241):
    for _ in range(n):r=g.update(p,v,**c)
    return r

class Rubric(unittest.TestCase):
    def test_folded_released_supported_full_second(self):
        p,v,c=state();r=hold(DryerGrade(),p,v,c)
        self.assertEqual(r['score'],1.);self.assertTrue(r['finished']);self.assertAlmostEqual(r['stable_seconds'],1.)
    def test_real_hinge_pin_support_counts(self):
        p,v,c=state(90,65);r=hold(DryerGrade(),p,v,c)
        self.assertEqual(r['score'],1.);self.assertEqual(r['support_primitive'],'head/hinge_pin')
    def test_real_handle_support_at_other_roll_counts(self):
        p,v,c=state(90,20);r=hold(DryerGrade(),p,v,c)
        self.assertEqual(r['score'],1.);self.assertEqual(r['support_primitive'],'handle/grip')
    def test_hidden_hinge_assembly_penetration_does_not_count(self):
        p,v,c=state(90,65);p[:,:3]-=[0,0,.005]
        r=hold(DryerGrade(),p,v,c);self.assertFalse(r['supported']);self.assertNotEqual(r['score'],1.)
    def test_idle_and_half_fold_not_complete(self):
        for angle,expected in [(0,0.),(55,.4)]:
            p,v,c=state(angle);r=hold(DryerGrade(),p,v,c)
            self.assertEqual(r['score'],expected);self.assertFalse(r['finished'])
    def test_240_samples_are_less_than_a_second(self):
        p,v,c=state();r=hold(DryerGrade(),p,v,c,240)
        self.assertEqual(r['score'],.7);self.assertLess(r['stable_seconds'],1.)
    def test_touching_handle_or_any_cord_is_not_release(self):
        for j in [L.HEAD,L.HANDLE,2,5,L.BUNDLE]:
            p,v,c=state();c['robot_contact_counts'][0,j]=1
            r=hold(DryerGrade(),p,v,c);self.assertEqual(r['score'],.7);self.assertFalse(r['free'])
    def test_supported_not_hovering(self):
        p,v,c=state();p[:,:3]+=[0,0,.025]
        r=hold(DryerGrade(),p,v,c);self.assertFalse(r['supported']);self.assertNotEqual(r['score'],1.)
    def test_motion_restarts_hold(self):
        p,v,c=state();g=DryerGrade();hold(g,p,v,c,230);v[0,0]=.020
        r=g.update(p,v,**c);self.assertEqual(r['stable_seconds'],0.)
        v[0,0]=0.;r=hold(g,p,v,c,240);self.assertNotEqual(r['score'],1.)
    def test_slow_hinge_drift_is_not_settled(self):
        # A slowly reopening hinge can satisfy every instantaneous velocity
        # threshold. Its actual angle must also remain stable over the hold.
        g=DryerGrade()
        for i in range(720):
            p,v,c=state(angle=85.-.4*i/240.)
            r=g.update(p,v,**c)
            self.assertNotEqual(r['score'],1.)
        self.assertTrue(r['folded']);self.assertTrue(r['quiet'])
    def test_disconnected_hinge_is_terminal_failure(self):
        p,v,c=state();p[L.HANDLE,0]+=.02;r=hold(DryerGrade(),p,v,c)
        self.assertEqual(r['score'],0.);self.assertEqual(r['void'],'folding hinge separated')
    def test_detached_cord_is_terminal_failure(self):
        p,v,c=state();p[4,0]+=.02;r=hold(DryerGrade(),p,v,c)
        self.assertEqual(r['score'],0.);self.assertEqual(r['void'],'cable detached')
    def test_wrong_hinge_axis_is_terminal_failure(self):
        p,v,c=state();p[L.HANDLE,3:]=Rotation.from_euler('x',10,degrees=True).as_quat()
        r=hold(DryerGrade(),p,v,c);self.assertEqual(r['score'],0.);self.assertIsNotNone(r['void'])
    def test_off_counter_cannot_recover(self):
        p,v,c=state();p[L.BUNDLE,0]=1.;g=DryerGrade();g.update(p,v,**c)
        p,v,c=state();r=hold(g,p,v,c);self.assertEqual(r['score'],0.);self.assertEqual(r['void'],'a component left the counter')
    def test_invalid_contacts_fail_closed(self):
        p,v,c=state();c['body_contact_forces'][0,0,0]=np.nan
        r=hold(DryerGrade(),p,v,c);self.assertEqual(r['score'],0.);self.assertIsNotNone(r['void'])

if __name__=='__main__':unittest.main()
