"""Real-scale desktop lever blade snapper; mechanism dimensions are estimates."""
import numpy as np
CONTROL_HZ=20
PHYSICS_HZ=960
MAX_STEPS=1600
BOUNCE_THRESHOLD=.20
INSTRUCTION='Refresh the exposed dull end segment of this 9 mm utility knife using the desktop lever blade snapper. Its original dock is on the table, the slider is open and the correct M sheath guide is already selected. Insert the steel sheath fully, hold the knife steady and press the front lever to snap off the dull segment inside the disposal box. Withdraw the refreshed knife, return it flat to the cutting mat and release both tools. Keep the broken steel segment inside the box throughout.'
LINKS=('blade_segment','case','knife','lever')
SEGMENT,CASE,KNIFE,LEVER=range(4)
BLADE_WIDTH=.009
BLADE_THICKNESS=.00038
SEGMENT_LENGTH=.005
SCORE_X=.003
SCORE_SLOPE=1/np.sqrt(3)
HANDLE_END=-.115
BREAK_TORQUE=.020
BREAK_FORCE=1000.
KNIFE_START=np.array((.43,.29,.0068))
CASE_START=np.array((.36,.14,0.))
HINGE_LOCAL=np.array((0.,-.006,.072))
SEAT_LOCAL=np.array((0.,0.,.109))
GUIDE_HALF=np.array((.0075,.004))
# Knife axis points down, its broad-face normal toward the left arm.
SEAT_ROTATION=np.array(((0.,-1.,0.),(0.,0.,1.),(-1.,0.,0.)))
LEVER_PRESS_LOCAL=np.array((-.023,-.027,.106))
START_POSES=np.array([(*(KNIFE_START+[SCORE_X+SEGMENT_LENGTH/2,0,0]),0,0,0,1),(*CASE_START,0,0,0,1),(*KNIFE_START,0,0,0,1),(*(CASE_START+HINGE_LOCAL),0,0,0,1)],float)
RESET_POSES=dict(zip(LINKS,START_POSES))
MASSES=np.array((.00013338,.112,.021,.008))
MAT_CENTER=np.array((.40,.31,0))
MAT_HALF=np.array((.150,.100))
INNER_HALF=np.array((.029,.017))
INNER_FLOOR=.004
INNER_TOP=.112
TABLE_BOUNDS=(-.10,.85,-.50,.50)
def quat_matrix(q):
 x,y,z,w=np.asarray(q,dtype=float)
 return np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
def segment_vertices():
 return np.array([(x+SCORE_SLOPE*y,y,z) for x in (-SEGMENT_LENGTH/2,SEGMENT_LENGTH/2) for y in (-BLADE_WIDTH/2,BLADE_WIDTH/2) for z in (-BLADE_THICKNESS/2,BLADE_THICKNESS/2)])
