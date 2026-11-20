"""True-size consumer pushchair wheel module; unpublished detail estimates explicit."""
import numpy as np
CONTROL_HZ=20
MAX_STEPS=700
INSTRUCTION="Remove the stroller rear wheel so its axle and wheel hub can be serviced. The frame is already safely supported in the bench clamp. Hold the lower release lever, support the full-size wheel while withdrawing its steel axle completely, and put the wheel upright in the empty maintenance rack. Release the lever and both hands, leaving the wheel stable and the spring retainer returned. Cleaning or refitting the wheel is a subsequent operation, not part of this task."
LINKS=('old_axle','old_wheel','release_slider')
OLD_AXLE,OLD_WHEEL,PIN=range(3)
CENTER=np.array((.36,0.,.150))
NEW_START=np.array((.55,.270,.131))
OLD_CRADLE=np.array((.51,-.245,.131))
RADIUS=.120
TIRE_INNER=.105
TIRE_WIDTH=.038
WHEEL_MASS=.58
AXLE_MASS=.110
PIN_MASS=.025
PIN_CENTER=CENTER+(.085,0.,-.016)
PIN_TOP_LOCAL=.0117  # top = axis -4.3 mm at the slider's upper stop
PIN_STROKE=.008
PIN_SPRING=400.
PIN_TARGET=.001
PIN_DAMPING=2.
TAB_LOCAL=np.array((0.,.052,-.006))
AXLE_RADIUS=.006
GROOVE_RADIUS=.004
GROOVE_START=.0825
GROOVE_END=.0875
AXLE_END=.121
BORE_FRONT=CENTER[0]+.060
BORE_BACK=CENTER[0]+.125
BORE_RADIUS=.00630
CRADLE_FLOOR=.004
CRADLE_INNER_X=.043
CRADLE_LENGTH_Y=.255
OLD_CRADLE_CHEEK_HEIGHT=.090
NEW_CRADLE_CHEEK_HEIGHT=.052
TABLE_BOUNDS=np.array(((-.19,.91),(-.65,.65)))
CONTACT_OFFSET=.00015
LINEAR_DAMPING=.01
ANGULAR_DAMPING=.03
BOUNCE_THRESHOLD=.20
MASSES=np.array((AXLE_MASS,WHEEL_MASS,PIN_MASS))
START_POSES=np.array([np.r_[CENTER,(0,0,0,1)],np.r_[CENTER,(0,0,0,1)],np.r_[PIN_CENTER,(0,0,0,1)]],dtype=float)
RESET_POSES=START_POSES

def quat_matrix(q):
 x,y,z,w=np.asarray(q,dtype=float)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def pin_travel(p):return float(p[PIN,2]-PIN_CENTER[2])

def rx(t):
 c,s=np.cos(t),np.sin(t);return np.array(((1,0,0),(0,c,-s),(0,s,c)))
