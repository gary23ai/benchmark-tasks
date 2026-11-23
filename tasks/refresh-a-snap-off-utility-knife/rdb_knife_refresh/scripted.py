"""Privileged reference, issuing only stock robot joint/jaw targets."""
import json
import numpy as np
from inspect_robots import Action,ActionChunk
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from . import layout as L
from .fixed_scene import ARMS,BASES,CONTROL_HZ
OPEN=float(FINGER_OPEN)


def frame(approach,closing):
    z=np.asarray(approach,dtype=float);z/=np.linalg.norm(z)
    x=np.asarray(closing,dtype=float);x-=z*(x@z);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))


class KnifeReference:
    def __init__(self):
        self.joints={a:np.asarray(READY,dtype=float).copy() for a in ARMS}
        self.aperture=dict.fromkeys(ARMS,OPEN);self.points={};self.rotations={};self.state=None;self.measured=None
        self.tick=0;self.phases=[]

    def observe(self,joint_pos,demo_state):self.measured=np.asarray(joint_pos).copy();self.state=demo_state

    def phase(self,name):
        self.phases.append([self.tick,name]);print('KNIFE_PHASE '+json.dumps(dict(tick=self.tick,phase=name,
            state=None if self.state is None else self.state['poses'].tolist())),flush=True)

    def emit(self):
        self.tick+=1
        target=np.concatenate([np.r_[self.joints[a],np.clip(self.aperture[a]/OPEN,0.,1.)] for a in ARMS])
        return ActionChunk(actions=[Action(data=target,meta={})],control_hz=CONTROL_HZ)

    def hold(self,seconds):
        for _ in range(max(1,round(seconds*CONTROL_HZ))):yield self.emit()

    def solve(self,arm,point,rotation):
        side=1 if arm=='left' else -1
        seeds=[self.joints[arm],np.array((0.,1.6,1.4,-.8,0.,0.)),np.array((side*.4,2.2,1.,-.5,side,0.)),np.array((0.,2.1,1.,-.4,side*1.5,-.1))]
        best=None
        for seed in seeds:
            q,pe,re=ik.solve(point,ik.matrix_to_quat_xyzw(rotation),seed,BASES[arm],iters=100)
            if best is None or pe+.01*re<best[1]+.01*best[2]:best=q,pe,re
            if pe<.0002 and re<.01:break
        q,pe,re=best
        if pe>.003 or re>.06:
            # Bound-constrained least squares avoids a clipped DLS local
            # optimum at the real joint stops. Actual spindle alignment is
            # separately controlled from its measured axis below.
            from scipy.optimize import least_squares
            from scipy.spatial.transform import Rotation
            def residual(candidate):
                actual,orientation=ik.tcp_pose(candidate,BASES[arm])
                return np.r_[actual-np.asarray(point),.05*Rotation.from_matrix(rotation@orientation.T).as_rotvec()]
            fitted=least_squares(residual,np.clip(q,ik.LOWER+1e-9,ik.UPPER-1e-9),
                bounds=(ik.LOWER,ik.UPPER),max_nfev=200,ftol=1e-11,gtol=1e-11,xtol=1e-11)
            actual,orientation=ik.tcp_pose(fitted.x,BASES[arm])
            retry=(fitted.x,float(np.linalg.norm(actual-point)),float(np.linalg.norm(Rotation.from_matrix(rotation@orientation.T).as_rotvec())))
            if retry[1]+.01*retry[2]<pe+.01*re:q,pe,re=retry
        # Orientation is a soft objective at a real wrist stop. The task
        # still requires the actual spindle to align within three degrees.
        if pe>.003 or re>getattr(self,'rotation_tolerance',.20):raise RuntimeError(f'{arm} unreachable {point}: {pe}m/{re}rad')
        return q

    def go(self,arm,point,rotation,seconds):
        before=self.joints[arm].copy();target=self.solve(arm,point,rotation)
        count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);self.joints[arm]=before+f*(target-before);yield self.emit()
        self.points[arm]=np.asarray(point,dtype=float).copy();self.rotations[arm]=np.asarray(rotation).copy()

    def line(self,arm,point,seconds,gap=None):
        start=self.points[arm].copy();old=self.aperture[arm];count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);self.points[arm]=start+f*(np.asarray(point)-start)
            actual,_=ik.tcp_pose(self.measured[:6] if arm=='left' else self.measured[7:13],BASES[arm])
            correction=np.clip(.7*(self.points[arm]-actual),-.012,.012)
            self.joints[arm]=self.solve(arm,self.points[arm]+correction,self.rotations[arm])
            if gap is not None:self.aperture[arm]=old+f*(gap-old)
            yield self.emit()

    def center(self,body):return np.asarray(self.state['poses'][body,:3]).copy()

    def capture(self,arm,body):
        pose=self.state['poses'][body];r=L.quat_matrix(pose[3:])
        self.grips=getattr(self,'grips',{})
        q=self.measured[:6] if arm=='left' else self.measured[7:13]
        actual,orientation=ik.tcp_pose(q,BASES[arm])
        self.grips[arm]=(r.T@(actual-pose[:3]),r.T@orientation)
        self.grip_biases=getattr(self,'grip_biases',{})
        self.grip_biases[arm]=(self.points[arm]-actual,self.rotations[arm]@orientation.T)

    def carry(self,arm,body,center,rotation,seconds):
        self.rotation_tolerance=.45
        self.capture(arm,body)
        start=self.center(body);r0=L.quat_matrix(self.state['poses'][body,3:]);local,relative=self.grips[arm]
        position_bias,rotation_bias=self.grip_biases[arm]
        count=max(1,round(seconds*CONTROL_HZ));offset=np.zeros(3)
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);wanted=start+f*(np.asarray(center)-start)
            r=ik.slerp_matrix(r0,rotation,f)
            offset+=.20*np.clip(wanted-self.center(body),-.001,.001)
            self.points[arm]=wanted+r@local+position_bias+offset;self.rotations[arm]=rotation_bias@r@relative
            self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm]);yield self.emit()
        self.rotation_tolerance=.20

    def pose_hold(self,arm,body,center,rotation,seconds):
        from scipy.spatial.transform import Rotation
        point=self.points[arm].copy();tcp_rotation=self.rotations[arm].copy()
        for _ in range(round(seconds*CONTROL_HZ)):
            point+=.35*np.clip(np.asarray(center)-self.center(body),-.001,.001)
            actual=L.quat_matrix(self.state['poses'][body,3:])
            turn=.35*Rotation.from_matrix(rotation@actual.T).as_rotvec();magnitude=np.linalg.norm(turn)
            if magnitude>.005:turn*=.005/magnitude
            tcp_rotation=Rotation.from_rotvec(turn).as_matrix()@tcp_rotation
            self.points[arm]=point.copy();self.rotations[arm]=tcp_rotation.copy()
            self.joints[arm]=self.solve(arm,point,tcp_rotation);yield self.emit()

    def insert_sheath(self):
        # Follow the actual free box, using measured tool pose. Only robot
        # targets move; the real guide walls support and align the sheath.
        target_z=.139
        for i in range(260):
            case=self.state['poses'][L.CASE];cr=L.quat_matrix(case[3:])
            local=(self.center(L.KNIFE)-case[:3])@cr
            lateral=np.linalg.norm(local[:2]);kr=L.quat_matrix(self.state['poses'][L.KNIFE,3:])
            segment=self.state['poses'][L.SEGMENT];br=L.quat_matrix(segment[3:])
            corners=(segment[:3]+L.segment_vertices()@br.T-case[:3])@cr
            # Actual eight-corner clearance through the shoulder ledges and
            # in front of the open striker, including 0.2 mm margin.
            fits=bool(np.max(abs(corners[:,0]))<.0058 and corners[:,1].min()>-.0021 and corners[:,1].max()<.0038)
            if lateral<.0010 and np.linalg.norm(kr[:,0]+cr[:,2])<.045 and fits:target_z=max(.109,target_z-.00025)
            wanted=case[:3]+cr@np.array((0,0,target_z))
            yield from self.pose_hold('left',L.KNIFE,wanted,cr@L.SEAT_ROTATION,.05)
            if i%20==0:print('SHEATH_INSERT '+json.dumps(dict(tick=self.tick,local=local.tolist(),target_z=target_z)),flush=True)
            if target_z<=.109 and abs(local[2]-.109)<.0008 and lateral<.0010:return
        raise RuntimeError('Actual sheath did not reach its physical guide depth stop')

    def sequence(self):
        above=frame((np.sin(np.deg2rad(10)),0,-np.cos(np.deg2rad(10))),(0,-1,0));identity=np.eye(3)
        self.phase('pick up the locked knife by its real steel handle')
        point=self.center(L.KNIFE)+[-.045,0,.013]
        yield from self.go('left',point+[0,0,.15],above,1.2)
        yield from self.line('left',point,1.8)
        yield from self.line('left',point,2.,gap=-.0045)
        yield from self.hold(.4)
        yield from self.carry('left',L.KNIFE,self.center(L.KNIFE)+[0,0,.18],identity,1.8)
        if self.center(L.KNIFE)[2]<.12:raise RuntimeError('Knife not physically lifted')
        self.phase('orient the sheath above the selected desktop guide')
        yield from self.carry('left',L.KNIFE,L.CASE_START+[0,0,.230],identity,2.)
        yield from self.carry('left',L.KNIFE,L.CASE_START+[0,0,.230],L.SEAT_ROTATION,3.)
        yield from self.pose_hold('left',L.KNIFE,L.CASE_START+[0,0,.230],L.SEAT_ROTATION,1.5)
        yield from self.carry('left',L.KNIFE,L.CASE_START+[0,0,.139],L.SEAT_ROTATION,2.)
        self.phase('insert the steel sheath to its real shoulder stop')
        yield from self.insert_sheath()
        self.phase('press the real hinged lever while holding the knife')
        press=frame((0,1,0),(1,0,0))
        case=self.state['poses'][L.CASE];cr=L.quat_matrix(case[3:]);contact=case[:3]+cr@L.LEVER_PRESS_LOCAL
        self.aperture['right']=0.
        yield from self.go('right',contact+[0,-.060,0],press,2.)
        yield from self.line('right',contact+[0,-.016,0],1.5)
        start=self.points['right'].copy()
        for i in range(120):
            yield from self.line('right',start+np.array((0,min(.026,(i+1)*.0003),0)),.05)
            if self.state['native_score_broken']:break
        if not self.state['native_score_broken']:raise RuntimeError('The real lever did not fracture the native scored joint')
        yield from self.line('right',self.points['right']+[0,-.080,0],1.5)
        yield from self.hold(1.)
        self.phase('withdraw the refreshed knife without removing the fragment')
        case=self.state['poses'][L.CASE];cr=L.quat_matrix(case[3:])
        yield from self.carry('left',L.KNIFE,self.center(L.KNIFE)+cr[:,2]*.11,cr@L.SEAT_ROTATION,2.)
        yield from self.carry('left',L.KNIFE,(.43,.29,.20),identity,3.)
        self.phase('return the refreshed knife to the cutting mat and release')
        yield from self.carry('left',L.KNIFE,(.43,.29,.0074),identity,2.5)
        yield from self.line('left',self.points['left'],.8,gap=OPEN)
        yield from self.line('left',self.points['left']+[0,0,.12],1.2)
        while True:yield self.emit()
