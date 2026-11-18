"""Dimensions and rules of the 2 litre pump-action pressure sprayer. Pure NumPy.

Sprayer frame: origin at the middle of the tank's base, z up along the sprayer's axis, x and y as the table's
at the start. Angles "round the sprayer" are measured about z from +x, anticlockwise seen from above; metres.

Sources. Overall height, tank diameter and empty weight are the maker's figures for a Spear & Jackson 2LPAPS
(320 mm, 134 mm, 0.35 kg). Where the parts sit (pump knob on top, spray head with the nozzle on one side and
the trigger handle on the other, pull-up safety valve on the tank's shoulder) is scaled from the drawing in
that maker's user guide. Everything marked ESTIMATE is not a maker's figure.
"""

import numpy as np

# -- tank ---------------------------------------------------------------------------------------------------
TANK_R = .067                    # 134 mm across (maker)
BODY_TOP = .164                  # ESTIMATE (drawing): the straight wall ends here and the shoulder begins
NECK_R = .0385                   # ESTIMATE (drawing): the shoulder meets the spray head's collar at this radius
NECK_Z = .220
HEIGHT = .320                    # top of the pump knob, pump pushed down (maker)
WATER_L = .5                     # water left in the tank for a trial
TANK_MASS = .246 + WATER_L       # empty tank (0.35 kg less head, knob and valve) and its water, one rigid body
TANK_COM_Z = .050

# -- spray head (turns on the tank's neck thread) -------------------------------------------------------------
COLLAR_TOP_R = .036              # ESTIMATE (drawing): ribbed collar, 77 mm across at the bottom
COLLAR_TOP = .254
HEAD_R = .033
HEAD_TOP = .277
NOZZLE_Z = .262                  # ESTIMATE (drawing): spout and brass nozzle, one straight bar out of the head
NOZZLE_W = .012                  # 12 mm across (collider: square bar; drawn round)
NOZZLE_TIP_R = .078              # the nozzle's end is 78 mm from the sprayer's axis
HANDLE_W = .022                  # trigger handle, opposite the nozzle
HANDLE_BOTTOM = .212             # it hangs down to here, clear of the valve when the head turns
HEAD_MASS = .080
TURNS = 1.5                      # ESTIMATE: turns of the neck thread from tight to free
HEAD_FREE = TURNS * 2 * np.pi    # the head's joint stops here: the thread has let go
HEAD_DRAG = .35                   # N m, ESTIMATE: running torque of the thread and its seal (a hand-tight cap of this
                                 # size takes more to break loose); turning it takes a 5 N push at the nozzle
NOZZLE_START = np.radians(-5.)   # where the nozzle points at the start (away from the robot); handle toward it

# -- pump knob (the pump rod spins freely in the head; its stroke is not modelled) ------------------------------
STEM_R = .008
KNOB_R = .024                    # ESTIMATE (drawing): 48 mm across, 13 mm thick
KNOB_BOTTOM = .307
KNOB_MASS = .020

# -- pull-up safety (pressure release) valve on the shoulder ----------------------------------------------------
VALVE_AZ = np.radians(140.)      # toward the robot's left
VALVE_BASE_R = .055              # where its stem leaves the shoulder
VALVE_STEM = .004                # stem showing between the shoulder and the cap, valve shut
VALVE_CAP = .016                 # ESTIMATE: red cap 16 mm across (collider: square) ...
VALVE_CAP_LEN = .014             # ... and 14 mm long
VALVE_TRAVEL = .005              # ESTIMATE: it pulls out 5 mm
VALVE_OPEN = .003                # air escapes once it is out this far
VALVE_SPRING = (2., 4.)          # N, ESTIMATE: spring force shut and fully out
VALVE_MASS = .004

# -- pressure (one number; no air or liquid is simulated) -------------------------------------------------------
START_BAR = 2.                   # after about 25 pump strokes (maker's instruction); ESTIMATE of the pressure
VENT_TAU = 1.2                   # s, ESTIMATE: with the valve held out the pressure falls as exp(-t / 1.2 s)
LEAK_TAU = .3                    # s: once the seal has opened the air goes out at the thread
SAFE_BAR = .1                    # "no pressure": what is left after the valve has been held open until it stops venting
SEAL_OPEN = np.radians(10.)      # taken as the point where the neck seal starts to open, on the safe side: no
                                 # measurement says how far a real head turns before it vents, so any loosening past
                                 # this 10-degree tolerance (play in the grip, a judge reading marks) counts

# -- rubber mat under the sprayer -------------------------------------------------------------------------------
MAT = (.200, .200, .003)


def shoulder_z(radius):
    """Height of the shoulder's surface at ``radius`` from the axis (a straight cone between body and neck)."""
    return BODY_TOP + (TANK_R - radius) / (TANK_R - NECK_R) * (NECK_Z - BODY_TOP)


SHOULDER_ELEV = float(np.arctan2(TANK_R - NECK_R, NECK_Z - BODY_TOP))    # the surface normal's angle above level


def radial(azimuth, elevation=0.):
    return np.array((np.cos(azimuth) * np.cos(elevation), np.sin(azimuth) * np.cos(elevation), np.sin(elevation)))


def valve_axis():
    """Unit vector the valve pulls out along: square to the shoulder."""
    return radial(VALVE_AZ, SHOULDER_ELEV)


def valve_base():
    return np.array((VALVE_BASE_R * np.cos(VALVE_AZ), VALVE_BASE_R * np.sin(VALVE_AZ), shoulder_z(VALVE_BASE_R)))


def valve_cap_centre(lift=0.):
    return valve_base() + valve_axis() * (VALVE_STEM + VALVE_CAP_LEN / 2 + lift)


def nozzle_azimuth(head_angle):
    return NOZZLE_START + head_angle


def nozzle_point(head_angle, radius):
    a = nozzle_azimuth(head_angle)
    return np.array((radius * np.cos(a), radius * np.sin(a), NOZZLE_Z))


def pressure_step(bar, valve_lift, head_angle, dt):
    """The tank pressure one time step later: it falls while the valve is held out, and faster once the head
    has been turned past the point where its seal opens."""
    if head_angle >= SEAL_OPEN:
        bar *= float(np.exp(-dt / LEAK_TAU))
    elif valve_lift >= VALVE_OPEN:
        bar *= float(np.exp(-dt / VENT_TAU))
    return 0. if bar < .01 else float(bar)
