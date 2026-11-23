"""Boundary tests for the unserved food. Each standard serving otherwise earns full credit."""
import json
from pathlib import Path
import numpy as np
import pytest
from task_impl.food_integrity import FOOD,NEIGHBORS,FoodGuard,gripper_contacts
from task_impl.grading import Grade
from task_impl.scene import load_fixture

CASES=json.loads(Path(__file__).with_name("cases.json").read_bytes())

def run(event=None,when=0):
    grade=Grade(load_fixture());steps=0
    for phase in CASES[0]["phases"]:
        for _ in range(phase["ticks"]):
            values=phase["measurements"].copy()
            if event and steps==when:values.update(event)
            final=grade.update(values);steps+=1
    return final

@pytest.mark.parametrize("food",FOOD)
@pytest.mark.parametrize("side",("left","right"))
def test_transient_unloaded_and_light_food_contact_voids_whole_attempt(side,food):
    # An actual touching contact patch with zero force and a lightly loaded
    # pair are both invalid, even when the next recorded tick is completely clean.
    assert run({f"touch_{side}_{food}":1})["score"]==0
    assert run({f"{side}_{food}":.000002})["score"]==0

@pytest.mark.parametrize("food",NEIGHBORS)
def test_removed_tilted_and_pinned_neighbors_are_invalid(food):
    for key,value in (("shift",.024),("tilt",15.),("press",.25)):
        assert run({f"{key}_{food}":value})["score"]==0

def test_lane_only_cancels_credit_before_the_first_supported_lift():
    for n in ("slice1","slice7"):
        assert run({"lane_"+n:.0075})["score"]==0
        end=sum(p["ticks"] for p in CASES[0]["phases"])-1
        assert run({"lane_"+n:.02},when=end)["score"]==1
        assert run({"lane_"+n:.003,"shift_slice5":.005,"tilt_slice3":3})["score"]==1

def test_patch_slice_mapping_distinguishes_jaws_and_speculation():
    f=np.zeros(16);d=np.ones(16)*.0005;s=np.array([0,0,1,1,1,1]);c=np.array([0,1,0,0,1,0])
    assert gripper_contacts(f,d,c,s)==(False,False)
    d[0]=-.000001
    assert gripper_contacts(f,d,c,s)==(True,False)
    d[0]=.0005;f[1]=.000002
    assert gripper_contacts(f,d,c,s)==(False,True)

def test_press_duration_and_opposed_cut_normals():
    f=load_fixture();g=FoodGuard(f);actors={a["name"]:a for a in f["actors"]}
    poses=np.array([[*actors[n]["position"],0,0,0,1] for n in NEIGHBORS],float)
    g.reset(poses.copy());force=np.zeros((len(FOOD),5))
    force[2,2]=g.weight[0]+.001
    for _ in range(59):val=g.tick(poses,force,1/240)
    assert val["press_slice1"] < .25
    assert g.tick(poses,force,1/240)["press_slice1"]==pytest.approx(.25)
    force[:]=0
    for k,j in enumerate((0,6)):
        poses[j,:2]+=g.normals[k]*.008
    out=g.tick(poses,force,1/240)
    assert out["press_slice1"]==0
    assert out["lane_slice1"]==pytest.approx(.008)
    assert out["lane_slice7"]==pytest.approx(.008)

def test_one_tick_downward_strike_invalidates_even_without_a_gripper_touch():
    f=load_fixture();g=FoodGuard(f);actors={a["name"]:a for a in f["actors"]}
    poses=np.array([[*actors[n]["position"],0.,0.,0.,1.] for n in NEIGHBORS])
    g.reset(poses);force=np.zeros((len(FOOD),5))
    # Deliver slightly more than the momentum of a whole portion falling
    # through its thickness, in one actual 240 Hz physics step.
    force[2,2]=g.weight[0]+1.05*g.impact_scale[0]*240
    out=g.tick(poses,force,1/240)
    assert out["impact_slice1"]==pytest.approx(1.05)
    assert out["press_slice1"]<.25
    assert run({"impact_slice1":out["impact_slice1"]})["score"]==0
    force[:]=0
    for _ in range(12):out=g.tick(poses,force,1/240)
    assert out["impact_slice1"]==pytest.approx(0.)
