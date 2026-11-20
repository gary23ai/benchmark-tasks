"""Synthetic scoring edge cases; these do not claim native task success."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rdb_stroller_remove import layout as L
from rdb_stroller_remove.grading import WheelGrade

def state():return L.START_POSES.copy(),np.zeros((3,6)),np.zeros((18,3),int)
def step(g,p,v,c):return g.update(p,v,robot_counts=c)
def stored():
 g=WheelGrade();p,v,c=state();p[L.PIN,2]-=.0065;step(g,p,v,c)
 p[L.OLD_AXLE,0]-=.14;p[L.OLD_WHEEL,0]-=.14;step(g,p,v,c)
 p[L.OLD_AXLE,:3]=L.OLD_CRADLE;p[L.OLD_WHEEL,:3]=L.OLD_CRADLE;p[L.PIN,:3]=L.PIN_CENTER
 return g,p,v,c

def test_idle_is_zero():
 g=WheelGrade();p,v,c=state()
 for _ in range(500):r=step(g,p,v,c)
 assert r['score']==0 and not r['old_removed']

def test_pressing_only_release_does_not_complete_removal():
 g=WheelGrade();p,v,c=state();p[L.PIN,2]-=.0065
 for _ in range(500):r=step(g,p,v,c)
 assert r['score']==.25 and not r['old_removed']

def test_forcing_axle_without_release_is_invalid():
 g=WheelGrade();p,v,c=state();p[L.OLD_AXLE,0]-=.010;p[L.OLD_WHEEL,0]-=.010
 r=step(g,p,v,c);assert r['finished'] and r['score']==0 and 'forced' in r['void']

def test_partial_withdrawal_not_full_axial_clearance():
 g=WheelGrade();p,v,c=state();p[L.PIN,2]-=.0065;step(g,p,v,c)
 p[L.OLD_AXLE,0]-=.04;p[L.OLD_WHEEL,0]-=.04
 r=step(g,p,v,c);assert not r['old_removed'] and r['score']==.25

def test_held_wheel_and_held_pin_cannot_finish():
 for body in (L.OLD_WHEEL,L.PIN):
  g,p,v,c=stored();c[0,body]=1
  for _ in range(500):r=step(g,p,v,c)
  assert r['score']==.75 and r['stable_seconds']==0 and not r['released']

def test_unreturned_retainer_prevents_completion():
 g,p,v,c=stored();p[L.PIN,2]-=.002
 for _ in range(500):r=step(g,p,v,c)
 assert r['score']==.75 and not r['retainer_returned']

def test_rack_endpoint_one_second_uses_241_samples():
 g,p,v,c=stored()
 for _ in range(240):r=step(g,p,v,c)
 assert r['score']==.75 and r['stable_seconds']==239/240
 r=step(g,p,v,c);assert r['score']==1 and r['stable_seconds']==1.

def test_wheel_below_bench_is_invalid_even_after_removal():
 g,p,v,c=stored();p[L.OLD_AXLE,2]=.04;p[L.OLD_WHEEL,2]=.04
 r=step(g,p,v,c);assert r['finished'] and r['score']==0 and 'fell' in r['void']

def test_separated_wheel_bearing_cannot_complete():
 g,p,v,c=stored();p[L.OLD_AXLE,0]+=.002
 r=step(g,p,v,c);assert r['void'] and 'bearing' in r['void']
