"""A loaf cake on a stand and a cutting wire with two wooden handles, on the dual-arm YAM station."""

from pathlib import Path

import numpy as np
from yam_arena import station
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens

from . import cake_model as cake
from .wire import CuttingWire

TASK_ID = "split-a-loaf-cake-into-two-layers-with-a-wire"
SCORER = "cake_cut_score"
USD = Path(__file__).with_name("cake.usda")
TOP = station.TABLE_TOP
if abs(TOP - cake.TABLE_TOP_WORLD_Z) > 1e-9:
    raise RuntimeError("cake_model.TABLE_TOP_WORLD_Z no longer matches the station's table top")
CONTROL_HZ = 20
PHYSICS_HZ = 240
SUCCESS_HOLD_S = 2
MAX_STEPS = 1600
ARMS = ("left", "right")
BASES = {arm: np.asarray(station.BASES[arm]) - (0., 0., TOP) for arm in ARMS}
PARK = {"left": (.14, .34, .19), "right": (.14, -.34, .19)}
ACTORS = cake.ACTORS
# World start poses (position, XYZW quaternion). The handles' cylinders are authored along x.
WORLD_START = {name: (*(cake.START[name] + (0., 0., TOP)), 0., 0., 0., 1.) for name in ACTORS}

# --- the wire model (wire.py) ---------------------------------------------------------------------------
WIRE_SEGMENTS = 64
# End to end. A real 0.64 mm steel wire is some 40 times stiffer; the difference is taken up, on a real
# robot, by the arms' own give (about 400 N/m at the fingertips).
WIRE_STIFFNESS = 1500.
CUT_CELL = .0025           # plan grid on which the cut is recorded
LINE_TOLERANCE = .005      # m from the scored line that still counts as on the line
KERF = .0015               # a wire this close to an earlier cut meets no cake
END_PULL_LIMIT = 40.       # N, the most any segment of the wire carries, and so the most it passes to a handle
# A handle nobody holds weighs 5 g, and the wire's pull on it is applied as an outside force once per physics
# step. That is only stable for a spring softer than about mass / step^2 = 290 N/m, so the pull on a loose
# handle is scaled down until the taut wire it hangs on is no stiffer than half of that. The whole wire, taut
# from end to end, is 1,500 N/m and gets about a tenth; a short taut length is stiffer in proportion (a snag
# three segments from the handle is 32,000 N/m) and gets less. A handle held between both finger pads gets
# the full pull.
LOOSE_STIFFNESS = .5 * cake.HANDLE_MASS * PHYSICS_HZ ** 2      # 144 N/m
# A loose handle is a cylinder lying on the table, and PhysX has no rolling resistance: the weight of the few
# millimetres of wire that hang from its middle down to the table (0.1 to 0.3 mN) rolled one across the table
# in 39 s. A real peg does not roll under that. Pulls on a loose handle below this are not passed on, and
# larger ones are reduced by it. ESTIMATE: 4 percent of the handle's weight.
HANDLE_REST_PULL = .04 * cake.HANDLE_MASS * 9.81               # 2 mN
# Uncut cake holds the top layer on the bottom one: a spring-damper whose strength falls with the uncut area.
WELD_STIFFNESS, WELD_DAMPING = 6000., 30.            # N/m, N s/m
WELD_TURN_STIFFNESS, WELD_TURN_DAMPING = 5., .02     # N m/rad, N m s/rad
WELD_STRENGTH = cake.CAKE_SHEAR_STRENGTH * float(cake.CAKE_SIZE[0] * cake.CAKE_SIZE[1])   # 50 N at full area
WELD_TURN_STRENGTH = WELD_STRENGTH * float(cake.CAKE_SIZE[0]) / 4.                         # 2.3 N m
SEATED = .002              # m; the top layer counts as still in place within this

INSTRUCTION = (
    "Split the loaf cake into two layers of equal height with the cutting wire. Pick up the wire by its two "
    "handles, hold it taut and draw it level through the cake along its length, half way up: 30 mm above the mat "
    "the cake stands on, where its sides are scored. Do not touch the cake with the grippers. Leave the cake where "
    "it is with the top layer still lying on the bottom one, and put the wire down on the table when you have "
    "finished."
)


def make_wire():
    return CuttingWire(length=cake.WIRE_LENGTH, segments=WIRE_SEGMENTS, stiffness=WIRE_STIFFNESS,
                       linear_mass=cake.WIRE_LINEAR_MASS, yield_per_m=cake.CAKE_YIELD,
                       viscous_per_m=cake.CAKE_VISCOUS, along=cake.CAKE_ALONG, half=cake.CAKE_HALF,
                       seam=cake.SEAM_LOCAL, top_layer_half=(*cake.CAKE_HALF[:2], cake.TOP_LAYER_H / 2), cell=CUT_CELL,
                       line_tolerance=LINE_TOLERANCE, kerf=KERF, radius=cake.WIRE_DIAMETER / 2,
                       max_tension=END_PULL_LIMIT)


def handle_force(pull, taut_run, held):
    """Force applied to a handle for the wire's pull on it where the wire leaves it (wire.step).

    ``taut_run`` is the number of taut segments between the handle and whatever the wire is pulling against.
    Never more than END_PULL_LIMIT, whoever holds the handle.
    """
    size = float(np.linalg.norm(pull))
    if size == 0.:
        return np.zeros(3)
    scale = min(1., END_PULL_LIMIT / size)
    if not held:
        stiffness = WIRE_STIFFNESS * WIRE_SEGMENTS / max(taut_run, 1)
        scale *= min(1., LOOSE_STIFFNESS / stiffness) * max(0., 1. - HANDLE_REST_PULL / min(size, END_PULL_LIMIT))
    return pull * scale


def limited(forces, limit=END_PULL_LIMIT):
    """Per-node forces scaled together so that their sum is no more than ``limit``."""
    return forces * min(1., limit / max(float(np.linalg.norm(forces.sum(axis=0))), 1e-9))


def floor(points):
    """Height of the surface under each point: the stand or the table."""
    on_stand = np.all(np.abs(points[:, :2] - cake.STAND_CENTRE[:2]) < cake.STAND_SIZE[:2] / 2, axis=1)
    return np.where(on_stand, cake.STAND_TOP, 0.)


def wire_start():
    """The slack wire between the handles, bowed away from the robot, before it is let down onto the table."""
    a, b = (cake.HANDLE_START[arm] for arm in ARMS)
    s = np.linspace(0., 1., WIRE_SEGMENTS + 1)
    span = np.linalg.norm(b - a)
    sag = np.sqrt(3. * span * (cake.WIRE_LENGTH - span) / 8.) * .97
    nodes = a + s[:, None] * (b - a)
    nodes[:, 0] += 4. * sag * s * (1. - s)
    return nodes


# At reset the slack wire is let down onto the table before the trial starts. The wire model has no inertia
# and its step is implicit, so this is done in long steps: 600 of half a second, after which the wire has
# stopped moving (600 steps of 1/240 s, as first built, left the ends still sinking for minutes).
SETTLE_STEPS, SETTLE_DT = 600, .5


def lay_wire(wire, capsules):
    """Put the wire where it lies at the start of a trial, at rest, between the handles at their start poses."""
    wire.reset(wire_start())
    whole = ((cake.CAKE_CENTRE, np.eye(3)), (cake.TOP_LAYER_CENTRE, np.eye(3)), True)
    for _ in range(SETTLE_STEPS):
        wire.step(cake.HANDLE_START["left"], cake.HANDLE_START["right"], whole, floor, SETTLE_DT, (), capsules)


def handle_capsule(position, rotation):
    """End points of a handle's axis (its cylinder is authored along its own x)."""
    reach = rotation[:, 0] * (cake.HANDLE_LENGTH / 2 - cake.HANDLE_RADIUS)
    return position - reach, position + reach


def side_camera():
    eye = np.array((.78, .44, TOP + .46))
    back = eye - (.33, 0., TOP + .06)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(22.))
