"""Native snap-off knife refresh using native scored fracture and a desktop lever snapper."""
from dataclasses import replace
from pathlib import Path
import numpy as np
from inspect_robots import StepResult
from inspect_robots.policy import PolicyBase
from yam_arena.constants import PHYSICS_DT
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from . import fixed_scene as F,layout as L
from .grading import KnifeGrade
CURRENT=None


def array(value):
    tensor=value.torch if hasattr(value,'torch') else value
    return tensor.detach().cpu().numpy().copy() if hasattr(tensor,'detach') else np.asarray(tensor).copy()


class KnifeArena(YamArenaEmbodiment):
    def __init__(self,*,media_dir=None):
        global CURRENT
        super().__init__(scene='station',device='cpu',object_jitter=0,control_hz=F.CONTROL_HZ,cam_width=320,cam_height=176)
        self.info=replace(self.info,name='yam_knife_refresh',environment_id='isaaclab-arena-0.3.1-yam-knife-refresh',
            supported_target_kinds=frozenset(),docs='Stock calibrated YAM station and real-scale snap-blade utility knife. Three stock RGB cameras and 14 joint/jaw readings.')
        self.media_dir=Path(media_dir) if media_dir else None
        self.recorder=None;self.demo_state=None;self.final_grade=None;self.diagnostics=[];self.tick=0
        self._raw_sim_step=None;self._collect_physics=False;CURRENT=self

    def _ensure_sim(self,goal):
        if self._env is not None:return
        from .recording import review_camera
        def make_env(embodiment,*,device,control_hz,**kwargs):
            import isaaclab.sim as sim_utils
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
            from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
            from isaaclab_arena.scene.scene import Scene
            objects=scenes.task_room('station',embodiment.rig.top_cam)
            objects.append(Object(name='retrieval_fixture',object_type=ObjectType.BASE,spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(F.USD))))
            def configure(cfg):
                cfg=scenes.env_cfg_callback(control_hz)(cfg)
                cfg.sim.device='cpu'
                cfg.sim.dt=1/L.PHYSICS_HZ
                cfg.decimation=round(L.PHYSICS_HZ/control_hz)
                cfg.sim.render_interval=cfg.decimation
                cfg.sim.physics.enable_ccd=True
                cfg.sim.physics.bounce_threshold_velocity=L.BOUNCE_THRESHOLD
                return cfg
            env=IsaacLabArenaEnvironment(name='yam_knife_refresh',scene=Scene(objects),embodiment=embodiment,env_cfg_callback=configure)
            return ArenaEnvBuilder(env,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()
        with review_camera():
            from yam_arena import scene as scenes
            stock=scenes.make_env;scenes.make_env=make_env
            try:super()._ensure_sim(None)
            finally:scenes.make_env=stock

    def _physics_views(self):
        import omni.physics.tensors as tensors
        import omni.usd
        from yam_arena.robot import asset_link_paths
        root='/World/envs/env_0';stage=omni.usd.get_context().get_stage()
        roots=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()=='Actors' and str(p.GetPath()).startswith(root+'/')]
        if len(roots)!=1:raise RuntimeError(f'Expected one knife-refresh fixture: {roots}')
        import carb.settings
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        from isaaclab.sim.utils.stage import get_current_stage_id
        self.physics=tensors.create_simulation_view('torch',stage_id=get_current_stage_id());self.physics.set_subspace_roots('/')
        self.actor_paths=[roots[0]+'/'+name for name in L.LINKS]
        self.body_view=self.physics.create_rigid_body_view(roots[0]+'/*')
        if list(self.body_view.prim_paths)!=self.actor_paths:raise RuntimeError(f'Unexpected body order {self.body_view.prim_paths}')
        links=asset_link_paths();self.link_names=list(links)
        self.robot_links=[root+'/'+a.title()+'Arm'+suffix for a in F.ARMS for suffix in links.values()]
        self.robot_bodies=self.physics.create_rigid_contact_view(self.robot_links,[self.actor_paths]*len(self.robot_links),max_contact_data_count=16384)
        self.body_contacts=self.physics.create_rigid_contact_view(self.actor_paths,[self.actor_paths]*len(self.actor_paths),max_contact_data_count=8192)
        self.all_ids=self._torch.arange(len(L.LINKS),dtype=self._torch.int32,device=self._base.device)
        self.physics_dt=float(getattr(self._base,'physics_dt',0.) or PHYSICS_DT)

    def _read(self):
        self.poses=array(self.body_view.get_transforms());self.poses[:,2]-=F.TOP
        self.velocities=array(self.body_view.get_velocities())
        if not (np.isfinite(self.poses).all() and np.isfinite(self.velocities).all()):raise RuntimeError('Nonfinite native retrieval state')

    def _publish_demo_state(self):
        self.demo_state=dict(poses=self.poses.copy(),velocities=self.velocities.copy(),robot_counts=self.contact_counts.copy(),native_score_broken=self.native_score_broken)

    def reset(self,scene,*,seed=None):
        from .recording import Recorder
        if self.recorder is not None:self.recorder.close();self.recorder=None
        self._collect_physics=False;self._ensure_sim(None);self._physics_views()
        torch=self._torch
        pose=torch.as_tensor(L.START_POSES,dtype=self.body_view.get_transforms().dtype,device=self._base.device).clone();pose[:,2]+=F.TOP
        ids=torch.arange(len(L.LINKS),dtype=torch.int32,device=self._base.device)
        # The only task-body writes are this episode reset.
        self.body_view.set_transforms(pose,ids);self.body_view.set_velocities(torch.zeros_like(self.body_view.get_velocities()),ids)
        observation=super().reset(scene,seed=seed)
        self._read();error=np.linalg.norm(self.poses[:,:3]-L.START_POSES[:,:3],axis=1)
        if error.max()>.006:raise RuntimeError(f'Body reset drift {error*1000} mm')
        if not np.allclose(array(self.body_view.get_masses()).reshape(-1),L.MASSES,rtol=.01):raise RuntimeError('Body mass mismatch')
        if set(observation.images)!={'top_cam','left_cam','right_cam'} or set(observation.state)!={'joint_pos'}:raise RuntimeError('Unexpected observation fields')
        self.grading=KnifeGrade(sample_hz=1/self.physics_dt);self.final_grade=None;self.tick=0;self.diagnostics=[]
        self.contact_wrenches={}
        self.contact_counts=np.zeros((len(self.robot_links),len(L.LINKS)),dtype=int)
        self.joint_pos=np.asarray(observation.state['joint_pos'],dtype=float).copy()
        self.native_score_broken=False;self.break_events=[]
        import omni.physx,omni.physx.bindings._physx as physx_bindings
        from pxr import PhysicsSchemaTools
        def joint_event(event):
            if event.type==int(physx_bindings.SimulationEvent.JOINT_BREAK):
                raw=event.payload['jointPath']
                path=str(PhysicsSchemaTools.decodeSdfPath(raw[0],raw[1]))
                self.break_events.append(dict(path=path,physics_sample=getattr(self,'_physics_sample_number',0)+1))
                if path.endswith('/scored_break'):self.native_score_broken=True
        self._joint_event_stream=omni.physx.get_physx_interface().get_simulation_event_stream_v2()
        self._joint_event_subscription=self._joint_event_stream.create_subscription_to_pop(joint_event)
        self._physics_sample_number=0
        self._publish_demo_state();self._install_substep_sampler()
        self.recorder=Recorder(self,self.media_dir) if self.media_dir else None
        if self.recorder is not None:self.recorder.start()
        return observation

    def _install_substep_sampler(self):
        if self._raw_sim_step is not None:return
        if getattr(self._base,'_physics_handles_decimation',False):raise RuntimeError('Retrieval scoring requires individual substeps')
        self._raw_sim_step=self._base.sim.step
        def sampled(*args,**kwargs):
            result=self._raw_sim_step(*args,**kwargs)
            # Pop native queued break events before grading this same substep.
            # Pumping events does not advance physics or change the joint.
            self._joint_event_stream.pump()
            if self._collect_physics:
                poses=array(self.body_view.get_transforms());poses[:,2]-=F.TOP
                velocities=array(self.body_view.get_velocities())
                counts=array(self.robot_bodies.get_contact_data(dt=self.physics_dt)[4]).reshape(len(self.robot_links),len(L.LINKS))
                self._substep_counts=np.maximum(self._substep_counts,counts)
                contact_data=self.body_contacts.get_contact_data(dt=self.physics_dt)
                body_counts=array(contact_data[4]).reshape(4,4)
                start=int(array(contact_data[5]).reshape(4,4)[L.SEGMENT,L.LEVER]);number=int(body_counts[L.SEGMENT,L.LEVER])
                cp=array(contact_data[1])[start:start+number].reshape(-1,3);cp[:,2]-=F.TOP
                cn=array(contact_data[2])[start:start+number].reshape(-1,3)
                cf=array(contact_data[0])[start:start+number].reshape(-1,1)
                snapper_contacts=np.concatenate((cp,cn,cf),axis=1)
                body_forces=array(self.body_contacts.get_contact_force_matrix(dt=self.physics_dt)).reshape(4,4,3)
                self.contact_wrenches=dict(body_contact_counts=body_counts.tolist(),body_contact_forces_n=body_forces.tolist(),
                    robot_body_force_n=array(self.robot_bodies.get_contact_force_matrix(dt=self.physics_dt)).reshape(len(self.robot_links),len(L.LINKS),3).tolist(),
                    native_score_broken=self.native_score_broken,native_break_events=self.break_events.copy(),snapper_contacts=snapper_contacts.tolist())
                self._latest=self.grading.update(poses,velocities,robot_counts=counts,body_counts=body_counts,body_forces=body_forces,native_score_broken=self.native_score_broken,snapper_contacts=snapper_contacts)
                self._physics_sample_number+=1
                if self.recorder is not None:self.recorder.physics_sample(poses,velocities,counts,self.contact_wrenches,self._latest)
                self._sample_count+=1
            return result
        self._base.sim.step=sampled

    def step(self,action):
        self.commanded_joint_pos=np.asarray(action.data,dtype=float).copy()
        self._substep_counts=np.zeros((len(self.robot_links),len(L.LINKS)),dtype=int);self._sample_count=0;self._latest=None;self._collect_physics=True
        try:observation=super().step(action).observation
        finally:self._collect_physics=False
        if self._sample_count!=self._base.cfg.decimation:raise RuntimeError(f'Wrong physics sample count {self._sample_count}')
        self._read();self.tick+=1;self.joint_pos=np.asarray(observation.state['joint_pos'],dtype=float).copy()
        self.contact_counts=self._substep_counts.copy()
        self.final_grade=self._latest;self._publish_demo_state()
        if self.tick%5==0:self.diagnostics.append(self._diagnose())
        terminal=bool(self.final_grade['finished'])
        if self.recorder is not None:self.recorder.step(self.tick,observation,self.final_grade,terminal=terminal or self.tick==F.MAX_STEPS)
        return StepResult(observation=observation,terminated=terminal,
            termination_reason='success' if self.final_grade['score']==1. else self.final_grade['void'] if terminal else None,
            info={'knife_grade':self.final_grade})

    def _diagnose(self):
        from yam_arena import ik
        robots={}
        for i,a in enumerate(F.ARMS):
            p,r=ik.tcp_pose(self.joint_pos[i*7:i*7+6],F.BASES[a]);wanted,_=ik.tcp_pose(self.commanded_joint_pos[i*7:i*7+6],F.BASES[a])
            robots[a]=dict(tcp_pos=p.tolist(),tcp_rotation=r.tolist(),jaw=float(self.joint_pos[i*7+6]),commanded_tcp_pos=wanted.tolist())
        return dict(tick=self.tick,**self.final_grade,robot=robots,poses=self.poses.tolist(),velocities=self.velocities.tolist(),
            contact_counts=self.contact_counts.tolist(),contact_wrenches=self.contact_wrenches)

    def close(self):
        global CURRENT
        if self.recorder is not None:self.recorder.close();self.recorder=None
        if self._raw_sim_step is not None and self._base is not None:self._base.sim.step=self._raw_sim_step;self._raw_sim_step=None
        self._joint_event_subscription=None
        if CURRENT is self:CURRENT=None
        super().close()


class KnifeScripted(PolicyBase):
    def bind(self,embodiment_info):pass
    def reset(self,scene):
        from .scripted import KnifeReference
        self.reference=KnifeReference();self.actions=self.reference.sequence()
    def act(self,observation):
        self.reference.observe(observation.state['joint_pos'],CURRENT.demo_state if CURRENT is not None else None)
        return next(self.actions)


def make_embodiment(**kwargs):return KnifeArena(**kwargs)


def make_scripted():
    from yam_arena.scripted import YamScriptedPolicy
    policy=KnifeScripted();policy.info=replace(YamScriptedPolicy().info,name='knife_refresh_scripted');return policy
