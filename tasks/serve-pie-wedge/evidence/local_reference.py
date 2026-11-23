"""Run a diagnostic dual-YAM rollout on the local PhysX build with no rendering."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
import ovstage
from ovphysx import PhysX
from ovphysx.types import TensorType as TT
from ovphysx.population import physics_population_descs
from ovphysx.samples.python_samples.schema_support import physx_schema_paths

import importlib.util
from yam_arena import constants,ik
def _isolated_module(path, name):
    """Load a private copy to keep the physical controller and other callers untouched."""
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def _fast_tcp(reference):
    """Precompute the fixed transforms on the URDF base-to-gripper branch only."""
    import numpy as np
    from yam_arena import urdf
    from yam_arena.constants import ARM_JOINTS, TCP_OFFSET
    joints=urdf.parse_joints(reference.YAM_URDF)
    parent_joint={j.child:j for j in joints.values()}
    chain=[]
    child="gripper"
    while child!="base":
        joint=parent_joint[child]
        origin=np.eye(4)
        origin[:3,:3]=urdf.rpy_matrix(joint.rpy)
        origin[:3,3]=joint.xyz
        if joint.type=="fixed":
            chain.append((origin,None,None,None))
        else:
            if joint.type not in {"revolute","continuous"} or joint.name not in ARM_JOINTS:
                raise ValueError("Unrecognized joint on the pinned YAM gripper chain")
            x,y,z=joint.axis/np.linalg.norm(joint.axis)
            skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            chain.append((origin,ARM_JOINTS.index(joint.name),skew,skew@skew))
        child=joint.parent
    chain.reverse()
    eye=np.eye(3)
    offset=np.array(TCP_OFFSET)

    def tcp(q,base_pos=(0.,0.,0.)):
        pose=np.eye(4)
        sines=np.sin(q)
        cosines=1-np.cos(q)
        step=np.eye(4)
        for origin,index,skew,square in chain:
            pose=pose@origin
            if index is not None:
                step[:3,:3]=eye+sines[index]*skew+cosines[index]*square
                pose=pose@step
        return pose[:3,:3]@offset+pose[:3,3]+np.asarray(base_pos),pose[:3,:3]
    return tcp

def _offline_solver(reference, kinematics):
    import numpy as np
    # Keep the exact upstream DLS iterations, limits, finite differences and seed ranking.
    # Only its FK subroutine changes. Never replace the globally loaded official module.
    private_ik=_isolated_module(reference.__file__,"robocurve_offline_yam_ik")
    fast=_fast_tcp(reference)
    for f in (0,.15,.5,.8,1.):
        pose=reference.LOWER+f*(reference.UPPER-reference.LOWER)
        original=reference.tcp_pose(pose)
        cached=fast(pose)
        if not all(np.allclose(a,b,atol=1e-12,rtol=0) for a,b in zip(original,cached)):
            raise ValueError("Cached gripper chain disagrees with official forward kinematics")
    private_ik.tcp_pose=fast
    private_policy=_isolated_module(kinematics.__file__,"robocurve_offline_policy_ik")
    private_policy.ik=private_ik
    return private_policy.solve_pose

TABLE=.7475

def attr(p,name,typ,value):
    p.CreateAttribute(name,typ,True).Set(value)

def arm(stage,name,y,yam_usd,parent="/Fixture",shift=(0.,0.,0.)):
    usdpath=parent+"/"+name
    robot=UsdGeom.Xform.Define(stage,usdpath);root=robot.GetPrim()
    root.GetReferences().AddReference(str(yam_usd.resolve()),"/yam")
    robot.AddTranslateOp().Set(Gf.Vec3d(0,y,.7500))
    root.AddAppliedSchema("PhysxArticulationAPI")
    joint=stage.GetPrimAtPath(usdpath+"/Physics/root_joint")
    joint.GetRelationship("physics:body0").SetTargets([])
    joint.GetAttribute("physics:localPos0").Set(Gf.Vec3f(shift[0],shift[1]+y,shift[2]+.7500))
    for k,v in (("solverPositionIterationCount",16),("solverVelocityIterationCount",4)):
        attr(root,"physxArticulation:"+k,Sdf.ValueTypeNames.Int,v)
    attr(root,"physxArticulation:enabledSelfCollisions",Sdf.ValueTypeNames.Bool,False)
    for i in range(1,9):
        p=stage.GetPrimAtPath(usdpath+"/Physics/joint"+str(i));p.AddAppliedSchema("PhysxJointAPI")
        k,d,eff,fr=(80,5,27,.3) if i<=3 else (10,1.5,7,.06) if i<=6 else (5000,100,50,0)
        rot=i<=6;scale=np.pi/180 if rot else 1
        for key,v in (("stiffness",k*scale),("damping",d*scale),("maxForce",eff)):
            attr(p,"drive:"+("angular" if rot else "linear")+":physics:"+key,Sdf.ValueTypeNames.Float,float(v))
        attr(p,"physxJoint:jointFriction",Sdf.ValueTypeNames.Float,float(fr))
    for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and not p.IsInstanceProxy():
            p.AddAppliedSchema("PhysxRigidBodyAPI")
            attr(p,"physxRigidBody:maxDepenetrationVelocity",Sdf.ValueTypeNames.Float,1.)
            attr(p,"physxRigidBody:disableGravity",Sdf.ValueTypeNames.Bool,True)

def scene(f,stage_text,out,sides,yam_usd):
    out.mkdir(parents=True,exist_ok=True);src=out/"scene.usda"
    src.write_text(stage_text,encoding="utf8")
    s=Usd.Stage.Open(str(src));ph=UsdPhysics.Scene.Define(s,"/Fixture/Physics")
    ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ph.CreateGravityMagnitudeAttr(9.81)
    pp=ph.GetPrim();pp.AddAppliedSchema("PhysxSceneAPI")
    for k,t,v in (("solverType",Sdf.ValueTypeNames.Token,"TGS"),("enableGPUDynamics",Sdf.ValueTypeNames.Bool,True),("broadphaseType",Sdf.ValueTypeNames.Token,"GPU")):
        attr(pp,"physxScene:"+k,t,v)
    for k,n in (("gpuFoundLostPairsCapacity",262144),("gpuFoundLostAggregatePairsCapacity",262144),("gpuTotalAggregatePairsCapacity",131072),("gpuMaxRigidContactCount",131072),("gpuMaxRigidPatchCount",32768)):
        attr(pp,"physxScene:"+k,Sdf.ValueTypeNames.UInt,n)
    if not f.get("pedestal_station"):
        table=UsdGeom.Cube.Define(s,"/Fixture/Bench");table.CreateSizeAttr(1)
        table.AddTranslateOp().Set(Gf.Vec3d(.52,0,TABLE-.019))
        table.AddScaleOp().Set(Gf.Vec3f(.8,1.55,.038))
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    for name,sg in sides.items():arm(s,name.title(),sg*.305,yam_usd)
    built=out/"assembled.usda";s.GetRootLayer().Export(str(built));return built

def main():
    ap=argparse.ArgumentParser();ap.add_argument("task",type=Path);ap.add_argument("out",type=Path)
    ap.add_argument("--yam-usd",type=Path,required=True);ap.add_argument("--control",choices=("reference","finger","lane","strike"),default="reference")
    a=ap.parse_args()
    sys.path.insert(0,str(a.task.resolve()))
    from task_impl.scene import compile_layer
    from task_impl import kinematics
    f=json.loads((a.task/"task_impl/fixture.json").read_bytes())
    if a.control!="reference":
        choice=json.loads(Path(__file__).with_name("control_actions.json").read_bytes())[a.control]
        if "trajectory" in choice:f["trajectory"]=choice["trajectory"]
        else:
            for index,waypoint in choice["replaced"].items():f["trajectory"][int(index)]=waypoint
        f["limit"]=choice["limit"]
    sgs={"left":1,"right":-1}
    built=scene(f,compile_layer(f),a.out,sgs,a.yam_usd)
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    PhysX.set_cpu_mode(False);stage=ovstage.Stage("fixture");engine=PhysX()
    ovstage.population.open_usd_with_desc(stage,str(built.resolve()),ordinal=1,time=ovstage.population.DEFAULT_TIME,descs=physics_population_descs(rendering=False))
    stage.advance_write_floor(ordinal=1).wait();engine.wait_task(engine.attach_ovstage(stage,read_ordinal=1))
    def bind(name,t):return engine.create_tensor_binding(pattern=name,tensor_type=t)
    def read(b):
        z=np.zeros(b.shape,np.float32);b.read(z);return z.reshape(-1,z.shape[-1])[0]
    def write(b,arr):b.write(np.asarray(arr,np.float32).reshape(b.shape))
    arms={};ready=[*constants.READY,constants.FINGER_OPEN,constants.FINGER_OPEN]
    for side,sg in sgs.items():
        name="/Fixture/"+side.title()
        po,ta,ro=[bind(name,tp) for tp in (TT.ARTICULATION_DOF_POSITION,TT.ARTICULATION_DOF_POSITION_TARGET,TT.ARTICULATION_ROOT_POSE)]
        write(ro,[0,sg*.305,.75,0,0,0,1]);write(po,ready);write(ta,ready)
        pv=bind(name,TT.ARTICULATION_DOF_VELOCITY);write(pv,np.zeros(pv.shape))
        arms[side]=(po,ta,pv)
    bodies={};body_vel={}
    for actor in f["actors"]:
        name="/Fixture/Actors/"+actor["name"]
        bb=bind(name,TT.RIGID_BODY_POSE);bv=bind(name,TT.RIGID_BODY_VELOCITY)
        write(bb,[*actor["position"][:2],actor["position"][2]+TABLE,0,0,0,1])
        if not actor["bolted"]:write(bv,[0]*6)
        bodies[actor["name"]]=bb;body_vel[actor["name"]]=bv
    def frame(name):
        if name=="table":return np.zeros(3),np.eye(3)
        z=read(bodies[name])
        return z[:3]-[0,0,TABLE],Rotation.from_quat(z[3:]).as_matrix()
    from task_impl.food_integrity import FOOD, NEIGHBORS, witness_points, world_marks, drifts, FoodGuard, gripper_contacts
    ws=witness_points(f)
    def pose_array():return np.stack([read(bodies[n]) for n in NEIGHBORS])
    arm_links=['base/link1/link2/link3/link4/link5/gripper'+suf for suf in ('','/tip_left','/tip_right')]
    filters={s:['/Fixture/'+s.title()+'/Geometry/'+link for link in arm_links] for s in sgs}
    tools=['/Fixture/Actors/blade','/Fixture/Actors/handle']
    views={}
    guard=FoodGuard(f)
    patches={}
    for food in FOOD:
        filtered=filters['left']+filters['right']+tools+['/Fixture/Actors/plate']
        if food in NEIGHBORS:filtered+=['/Fixture/Actors/crust','/Fixture/Actors/filling']
        sensor=engine.create_contact_binding(sensor_patterns=['/Fixture/Actors/'+food],filter_patterns=filtered,
            filters_per_sensor=len(filtered),max_contact_data_count=256)
        views[food]=(sensor,np.zeros((sensor.sensor_count,sensor.filter_count,3),np.float32))
        patches[food]=(np.zeros((256,1),np.float32),np.zeros((256,3),np.float32),np.zeros((256,3),np.float32),np.zeros((256,1),np.float32),np.zeros((1,sensor.filter_count),np.int32),np.zeros((1,sensor.filter_count),np.int32))
    def tick_guard():
        maximum={}
        inputs=[]
        for food,(sensor,buf) in views.items():
            sensor.read_normal_force_matrix(buf);vals=np.linalg.norm(buf[0],axis=-1)
            maximum['left_'+food]=float(sum(vals[:3]))
            maximum['right_'+food]=float(sum(vals[3:6]))
            down=max(0.,-float(buf[0,6,2]))+max(0.,-float(buf[0,7,2]))
            if food in NEIGHBORS:
                down+=sum(max(0.,-float(z)) for z in buf[0,9:,2])
                maximum['server_'+food]=float(sum(vals[6:8]))
            patch=patches[food];actual=sensor.read_normal_contact_data(*patch)
            if actual>=256:raise RuntimeError('Contact patch capacity')
            lt,rt=gripper_contacts(patch[0],patch[3],patch[4],patch[5])
            inputs.append([maximum['left_'+food],maximum['right_'+food],down,lt,rt])
            if food not in NEIGHBORS:
                maximum[food+'_blade']=float(vals[6]);maximum[food+'_plate']=float(vals[8])
        for name,v in zip(NEIGHBORS,drifts(pose_array(),ws,reference)):maximum['drift_'+name]=float(v)
        maximum.update(guard.tick(pose_array(),inputs,1/240))
        return maximum
    solve=_offline_solver(ik,kinematics);fk=_fast_tcp(ik)
    for _ in range(120):engine.step_sync(1/240)
    reference=world_marks(pose_array(),ws)
    guard.reset(pose_array())
    (a.out/"settled-reset.json").write_text(json.dumps({"joint":{s:read(arms[s][0]).tolist() for s in sgs},"poses":{n:read(b).tolist() for n,b in bodies.items()}},indent=2))
    print("Reset joints",{s:read(arms[s][0]).tolist() for s in sgs},flush=True)
    if any(not np.isfinite(read(arms[s][0])).all() for s in sgs):
        raise ValueError("Local articulation failed reset")
    goals={s:np.array(ready,dtype=float) for s in sgs}
    step=0;rows=[];horizon=f["limit"]
    for idx,w in enumerate(f["trajectory"]+[{"ticks":horizon,"left":None,"right":None}]):
        if step>=horizon:break
        st={s:read(arms[s][0]).astype(float) for s in sgs};last=goals;goals={}
        for s,sg in sgs.items():
            g=w[s];base=[0,.305*sg,.0025];goals[s]=st[s].copy()
            if idx==len(f["trajectory"]):goals[s]=last[s];st[s]=last[s];continue
            if not g:continue
            org,rot=frame(g["relative_to"]);tip=org+rot@np.array(g["tip"])
            if "held_actor" in g:
                xyz,mat=frame(g["held_actor"]);p,r=fk(st[s][:6],base)
                tip+=p+constants.PAD_TIP_PAST_TCP*r[:,2]-xyz-mat@np.asarray(g.get("held_point",[0,0,0]))
            target=kinematics.orientation(g["forward"],g["closing"])
            q,*_=solve(tip-constants.PAD_TIP_PAST_TCP*target[:,2],target,st[s][:6],base)
            goals[s]=np.r_[q,[-g["opening_m"]*.5]*2]
            if f.get("continuous_jaw") and last[s][-1]==goals[s][-1]:st[s][-2:]=last[s][-2:]
            if w.get("continue_arm_targets"):st[s][:6]=last[s][:6]
            if g.get("keep_arm_command"):goals[s][:6]=last[s][:6]
        for k in range(w["ticks"]):
            if step>=horizon:break
            t=(k+1)/w["ticks"];u=t*t*(3-2*t)
            for s in sgs:write(arms[s][1],st[s]*(1-u)+goals[s]*u)
            maxima={}
            for _ in range(12):
                engine.step_sync(1/240)
                latest=tick_guard()
                for name,value in latest.items():maxima[name]=max(value,maxima.get(name,value))

            step+=1
            if step>0:
                poses={c["name"]:read(bodies[c["name"]]).tolist() for c in f["actors"] if not c["bolted"]}
                for z in poses.values():z[2]-=TABLE
                rows.append(dict(step=step,waypoint=idx,poses=poses,guard=maxima,final_contact=latest,joints={s:read(arms[s][0]).tolist() for s in sgs}))
        print(step,{s:[round(a,4) for a in p[:3]] for s,p in rows[-1]["poses"].items()},flush=True)
        (a.out/"rollout.json").write_text(json.dumps(rows))
    engine.wait_task(engine.detach_ovstage());stage.destroy();engine.destroy()
if __name__=="__main__":main()
