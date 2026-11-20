"""Wheel-removal privileged reference, issuing only stock robot joint/jaw targets."""
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


class WheelReference:
    def __init__(self):
        self.joints={a:np.asarray(READY,dtype=float).copy() for a in ARMS}
        self.aperture=dict.fromkeys(ARMS,OPEN);self.points={};self.rotations={};self.state=None;self.measured=None
        self.tick=0;self.phases=[]

    def observe(self,joint_pos,demo_state):self.measured=np.asarray(joint_pos).copy();self.state=demo_state

    def phase(self,name):
        self.phases.append([self.tick,name]);print('WHEEL_PHASE '+json.dumps(dict(tick=self.tick,phase=name,
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
        if pe>.003 or re>.20:raise RuntimeError(f'{arm} unreachable {point}: {pe}m/{re}rad')
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

    def together(self,targets,seconds,gaps=None,smooth=True):
        starts={a:self.points[a].copy() for a in targets};old=self.aperture.copy();count=max(1,round(seconds*CONTROL_HZ))
        for i in range(1,count+1):
            f=i/count
            if smooth:f=f*f*(3-2*f)
            for a,dest in targets.items():
                self.points[a]=starts[a]+f*(np.asarray(dest)-starts[a])
                self.joints[a]=self.solve(a,self.points[a],self.rotations[a])
            for a,gap in (gaps or {}).items():self.aperture[a]=old[a]+f*(gap-old[a])
            yield self.emit()

    def orient(self,arm,rotation,seconds):
        point=self.points[arm].copy()
        yield from self.go(arm,point,rotation,seconds)

    def follow_body(self,arm,body,local,rotation,seconds):
        # Read-only task-body pose; ordinary joint targets remain the only action.
        pose=self.state['poses'][body];point=pose[:3]+L.quat_matrix(pose[3:])@np.asarray(local)
        yield from self.line(arm,point,seconds)

    def center(self,body):return np.asarray(self.state['poses'][body,:3]).copy()

    def body_line(self,arm,body,dest,seconds):
        yield from self.line(arm,self.points[arm]+np.asarray(dest)-self.center(body),seconds)

    def body_hold(self,arm,body,dest,seconds):
        point=self.points[arm].copy()
        for _ in range(round(seconds*CONTROL_HZ)):
            error=np.asarray(dest)-self.center(body)
            point+=.30*np.clip(error,-.002,.002)
            self.points[arm]=point.copy();self.joints[arm]=self.solve(arm,point,self.rotations[arm])
            yield self.emit()

    def supported_move(self,arm,body,dest,seconds):
        """Support the real wheel and keep its actual spindle axis square to the bore."""
        start=self.center(body);point=self.points[arm].copy();rotation=self.rotations[arm].copy()
        count=max(1,round(seconds*CONTROL_HZ));previous=start.copy()
        for i in range(1,count+1):
            f=i/count;f=f*f*(3-2*f);wanted=start+f*(np.asarray(dest)-start)
            error=wanted-self.center(body)
            point+=wanted-previous+.30*np.clip(error,-.004,.004);previous=wanted
            axis=L.quat_matrix(self.state['poses'][body,3:])[:,0]
            # Stop integrating inside one degree; otherwise gravity-induced
            # tiny residuals wind up the requested wrist angle at its stop.
            turn=.30*np.cross(axis,(1.,0.,0.)) if axis[0]<np.cos(np.deg2rad(1.)) else np.zeros(3)
            length=np.linalg.norm(turn)
            if length>.006:turn*=.006/length;length=.006
            if length>1e-12:
                k=turn/length;skew=np.array(((0,-k[2],k[1]),(k[2],0,-k[0]),(-k[1],k[0],0)))
                correction=np.eye(3)+np.sin(length)*skew+(1-np.cos(length))*(skew@skew)
                rotation=correction@rotation
            self.points[arm]=point.copy()
            self.joints[arm]=self.solve(arm,point,rotation)
            # Project the outer-loop rotation onto the achievable command.
            # This prevents integration beyond the real wrist hard stop.
            _,rotation=ik.tcp_pose(self.joints[arm],BASES[arm])
            self.rotations[arm]=rotation.copy()
            yield self.emit()

    def sequence(self):
        rr=frame((1,0,0),(0,0,-1));rl=frame((0,-1,0),(-1,0,0))
        tab=self.center(L.PIN)+L.TAB_LOCAL+(0.,.005,0.)
        self.phase('grasp the real lower release lever from its open outer side')
        yield from self.go('left',(.445,.137,.25),rl,1.2)
        yield from self.line('left',tab+(0,.080,.012),1.2)
        yield from self.line('left',tab,1.2)
        yield from self.line('left',tab,.7,gap=-.0075)
        yield from self.hold(.25)
        old=self.center(L.OLD_WHEEL)
        right_grip=old+(-L.TIRE_WIDTH/2+.009,-.052,0)
        self.phase('grasp the old full-size rear wheel by its spoke')
        yield from self.go('right',right_grip+(-.09,0,.015),rr,1.5)
        yield from self.line('right',right_grip,1.2)
        yield from self.line('right',right_grip,.7,gap=-.003)
        yield from self.hold(.3)
        self.phase('hold the spring retainer clear of the steel axle groove')
        target=L.PIN_CENTER+(0,0,-.0065)
        yield from self.body_line('left',L.PIN,target,1.0)
        yield from self.body_hold('left',L.PIN,target,1.0)
        self.phase('support the wheel and straighten its cantilevered spindle')
        yield from self.supported_move('right',L.OLD_AXLE,(self.center(L.OLD_AXLE)[0],0,L.CENTER[2]),1.)
        self.phase('withdraw the supported old wheel and axle completely')
        dest=np.array((L.CENTER[0]-.14,0,L.CENTER[2]))
        yield from self.supported_move('right',L.OLD_AXLE,dest,3.)
        yield from self.hold(.3)
        self.phase('release the unloaded spring lever')
        yield from self.line('left',self.points['left'],.6,gap=OPEN)
        yield from self.line('left',self.points['left']+(0,.08,.04),1.)
        self.phase('place the old wheel into the empty padded cradle')
        raised=self.center(L.OLD_WHEEL)+[0,0,.110]
        yield from self.supported_move('right',L.OLD_AXLE,raised,1.5)
        yield from self.supported_move('right',L.OLD_AXLE,raised,1.5)
        yield from self.supported_move('right',L.OLD_AXLE,np.r_[L.OLD_CRADLE[:2],raised[2]],2.5)
        self.phase('square the suspended old wheel before entering the narrow cradle')
        yield from self.supported_move('right',L.OLD_AXLE,np.r_[L.OLD_CRADLE[:2],raised[2]],2.5)
        yield from self.supported_move('right',L.OLD_AXLE,L.OLD_CRADLE,2.0)
        yield from self.supported_move('right',L.OLD_AXLE,L.OLD_CRADLE,.5)
        yield from self.line('right',self.points['right'],.7,gap=-.016)
        yield from self.line('right',self.points['right']+(-.08,0,.055),1.1)
        yield from self.line('right',self.points['right'],.4,gap=OPEN)
        self.phase('leave the unmounted wheel stable for axle and hub servicing')
        while True:yield self.emit()
