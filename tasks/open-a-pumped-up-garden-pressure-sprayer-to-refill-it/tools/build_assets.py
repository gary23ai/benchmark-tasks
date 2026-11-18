"""Regenerate ``rdb_sprayer_relief/sprayer.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs NumPy and SciPy.

The sprayer is one articulation rooted at the tank, which stands loose on a rubber mat: the spray head turns
on the tank (a hinge about the sprayer's axis with the thread's drag, 0 to 1.5 turns), the pump knob spins
freely on the head, and the safety valve's cap slides out of the shoulder against its spring. There is no
gauge: the 2LPAPS has none, and the tank's pressure is a number only the scorer reads. What is drawn is the product's outside
shape (lathe-turned meshes, ``collide=False``); what the physics sees are rigid convex hulls and boxes of the
same outside size, not drawn (``visible=False``). Parts of the one articulation do not collide with each other.
"""

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_sprayer_relief import fixed_scene as scene  # noqa: E402
from rdb_sprayer_relief import layout as L  # noqa: E402

# colour, roughness, metallic, friction, opacity
MATERIALS = {
    "tank_pe": ((.93, .93, .90), .45, 0., .6, .72),          # natural polyethylene, translucent
    "water": ((.35, .55, .80), .1, 0., .5, .55),
    "label": ((.10, .45, .22), .6, 0., .6, 1.),
    "head_black": ((.05, .05, .05), .5, 0., .8, 1.),
    "trigger_red": ((.78, .08, .06), .5, 0., .8, 1.),
    "valve_red": ((.85, .06, .05), .5, 0., .8, 1.),
    "brass": ((.80, .62, .25), .3, 1., .8, 1.),               # ESTIMATE: brass and plastic on the gripper's pads
    "steel": ((.62, .63, .65), .3, 1., .5, 1.),
    "mark_white": ((.98, .98, .98), .6, 0., .5, 1.),
    "mat_rubber": ((.10, .10, .11), .9, 0., 1.0, 1.),          # ESTIMATE: ribbed rubber mat under a wet plastic tank
}
_PREFIX = "</Fixture/Materials/"
SEGMENTS = 48


def _collide(material, offset):
    return (f"                rel material:binding:physics = {_PREFIX}{material}>\n"
            f"                float physxCollision:contactOffset = {offset:g}\n"
            "                float physxCollision:restOffset = 0\n")


def _fmt(points):
    return ", ".join(f"({x:.6g}, {y:.6g}, {z:.6g})" for x, y, z in points)


def mesh(name, material, points, faces, *, collide=False, visible=True, offset=.001):
    """A triangle/quad mesh in its body's frame. With ``collide`` it is also a rigid convex-hull collider."""
    apis = '"MaterialBindingAPI"' + (', "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI"' if collide else "")
    text = (f'            def Mesh "{name}" (\n                prepend apiSchemas = [{apis}]\n            )\n            {{\n'
            f"                rel material:binding = {_PREFIX}{material}>\n")
    if collide:
        text += _collide(material, offset) + '                uniform token physics:approximation = "convexHull"\n'
    if not visible:
        text += '                token visibility = "invisible"\n'
    return (text + "                uniform bool doubleSided = 1\n"
            f"                int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"                int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"                point3f[] points = [{_fmt(points)}]\n"
            '                uniform token subdivisionScheme = "none"\n'
            "            }\n\n")


def hull(name, material, points, *, collide=True, visible=True, offset=.001):
    """The convex hull of ``points`` (triangles facing out); a rigid convex-hull collider unless drawn only."""
    points = np.asarray(points, dtype=float)
    solid = ConvexHull(points)
    used = sorted(set(solid.simplices.ravel()))
    assert len(used) <= 64, (name, len(used))
    index = {old: new for new, old in enumerate(used)}
    centre = points[used].mean(axis=0)
    faces = []
    for a, b, c in solid.simplices:
        normal = np.cross(points[b] - points[a], points[c] - points[a])
        if normal @ (points[a] - centre) < 0:
            b, c = c, b
        faces.append((index[a], index[b], index[c]))
    return mesh(name, material, points[used], faces, collide=collide, visible=visible, offset=offset)


def ring(radius, z, count=24, phase=0.):
    return [(radius * math.cos(phase + 2 * math.pi * k / count), radius * math.sin(phase + 2 * math.pi * k / count), z)
            for k in range(count)]


def lathe(name, material, profile, *, rotation=None, origin=(0., 0., 0.), segments=SEGMENTS):
    """Drawn only: ``profile`` [(radius, z), ...] turned about z, then rotated and moved into the body's frame."""
    points, faces = [], []
    for r, z in profile:
        points += ring(r, z, segments)
    for i in range(len(profile) - 1):
        for k in range(segments):
            a, b = i * segments + k, i * segments + (k + 1) % segments
            faces.append((a, b, b + segments, a + segments))
    for i in (0, len(profile) - 1):                     # close the ends
        if profile[i][0] > 0:
            points.append((0., 0., profile[i][1]))
            for k in range(segments):
                faces.append((len(points) - 1, i * segments + k, i * segments + (k + 1) % segments))
    points = np.asarray(points)
    if rotation is not None:
        points = points @ np.asarray(rotation).T
    return mesh(name, material, points + np.asarray(origin), faces)


def bar(azimuth, r0, r1, width, z0, z1):
    """Corner points of a straight bar pointing out from the axis at ``azimuth``."""
    out, across = np.array((math.cos(azimuth), math.sin(azimuth), 0.)), np.array((-math.sin(azimuth), math.cos(azimuth), 0.))
    return [out * r + across * s * width / 2 + (0., 0., z) for r in (r0, r1) for s in (-1, 1) for z in (z0, z1)]


def swept(azimuth, outline, width):
    """Corner points of a flat piece: ``outline`` [(radius, z), ...] in the plane through the axis at ``azimuth``."""
    out, across = np.array((math.cos(azimuth), math.sin(azimuth), 0.)), np.array((-math.sin(azimuth), math.cos(azimuth), 0.))
    return [out * r + across * s * width / 2 + (0., 0., z) for r, z in outline for s in (-1, 1)]


def frame(axis, up_hint=(0., 0., 1.)):
    """Rotation whose local +z is ``axis`` and whose local +x is as close to ``up_hint`` as it can be."""
    z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    x = np.asarray(up_hint, dtype=float) - z * (np.asarray(up_hint) @ z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def wxyz(rotation):
    x, y, z, w = Rotation.from_matrix(np.asarray(rotation)).as_quat()
    return (w, x, y, z)


def body(name, mass, position, children, *, root=False, orient=(1., 0., 0., 0.), com=None, inertia=None):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = ("            bool physxArticulation:enabledSelfCollisions = 0\n"
                 "            int physxArticulation:solverPositionIterationCount = 32\n"
                 "            int physxArticulation:solverVelocityIterationCount = 8\n"
                 "            float physxArticulation:sleepThreshold = 0\n"
                 "            float physxArticulation:stabilizationThreshold = 0\n")
    if com is not None:
        extra += f"            point3f physics:centerOfMass = ({com[0]:.6g}, {com[1]:.6g}, {com[2]:.6g})\n"
    if inertia is not None:
        extra += f"            float3 physics:diagonalInertia = ({inertia[0]:.6g}, {inertia[1]:.6g}, {inertia[2]:.6g})\n"
    w, x, y, z = orient
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {mass:.6g}\n"
            "            bool physics:rigidBodyEnabled = 1\n" + extra +
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"            quatf xformOp:orient = ({w:.7g}, {x:.7g}, {y:.7g}, {z:.7g})\n"
            f"            double3 xformOp:translate = ({position[0]:.6g}, {position[1]:.6g}, {position[2]:.6g})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def _joint_frames(body0, body1, anchor0, rot0):
    w, x, y, z = rot0
    return (f"            rel physics:body0 = </Fixture/Actors/{body0}>\n"
            f"            rel physics:body1 = </Fixture/Actors/{body1}>\n"
            f"            point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            f"            quatf physics:localRot0 = ({w:.7g}, {x:.7g}, {y:.7g}, {z:.7g})\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n")


def hinge_z(name, body0, body1, anchor0, *, rot0=(1., 0., 0., 0.), limits=None, drag=0., stiffness=0., damping=None,
            armature=0.):
    """A hinge about body1's +z axis. ``drag`` (N m) is a velocity drive toward rest whose torque is capped: it
    resists turning with that torque and holds nothing at rest (Coulomb-like friction). ``stiffness`` and
    ``damping`` (N m per degree, N m s per degree) make a position drive instead."""
    if damping is None:
        damping = 2. * drag / math.degrees(.15)          # reaches the cap at 0.15 rad/s (9 degrees a second)
    force = f"            float drive:angular:physics:maxForce = {drag:.6g}\n" if drag else ""
    limit = (f"            float physics:lowerLimit = {limits[0]:.6g}\n            float physics:upperLimit = {limits[1]:.6g}\n"
             if limits else "")
    return (f'        def PhysicsRevoluteJoint "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI"]\n'
            "        )\n        {\n"
            f"            float drive:angular:physics:damping = {damping:.6g}\n" + force +
            f"            float drive:angular:physics:stiffness = {stiffness:.6g}\n"
            "            float drive:angular:physics:targetPosition = 0\n"
            "            float drive:angular:physics:targetVelocity = 0\n"
            '            uniform token drive:angular:physics:type = "force"\n'
            '            uniform token physics:axis = "Z"\n' + _joint_frames(body0, body1, anchor0, rot0) + limit +
            f"            float physxJoint:armature = {armature:g}\n"
            "            float physxJoint:maxJointVelocity = 720\n"
            "        }\n\n")


def slide_z(name, body0, body1, anchor0, rot0, upper, *, stiffness, target, damping, armature):
    """A slide along body1's +z axis with a spring (a position drive whose target lies behind the shut stop)."""
    return (f'        def PhysicsPrismaticJoint "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:linear", "PhysxJointAPI"]\n'
            "        )\n        {\n"
            f"            float drive:linear:physics:damping = {damping:.6g}\n"
            f"            float drive:linear:physics:stiffness = {stiffness:.6g}\n"
            f"            float drive:linear:physics:targetPosition = {target:.6g}\n"
            '            uniform token drive:linear:physics:type = "force"\n'
            '            uniform token physics:axis = "Z"\n' + _joint_frames(body0, body1, anchor0, rot0) +
            "            float physics:lowerLimit = 0\n"
            f"            float physics:upperLimit = {upper:.6g}\n"
            f"            float physxJoint:armature = {armature:g}\n"
            "        }\n\n")


def material(name):
    color, roughness, metallic, friction, opacity = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = {metallic}\n                float inputs:opacity = {opacity}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# ---------------------------------------------------------------------------------------------------------
def tank_parts():
    parts = []
    # what the physics sees: the straight wall and the shoulder, two convex pieces
    parts.append(hull("wall_collider", "tank_pe", ring(L.TANK_R, 0.) + ring(L.TANK_R, L.BODY_TOP), visible=False))
    parts.append(hull("shoulder_collider", "tank_pe", ring(L.TANK_R, L.BODY_TOP) + ring(L.NECK_R, L.NECK_Z), visible=False))
    # what is drawn: the bottle with a rounded foot and shoulder, the water in it, a label band, the neck mark
    parts.append(lathe("bottle", "tank_pe", [(L.TANK_R - .010, 0.), (L.TANK_R - .002, .004), (L.TANK_R, .012), (L.TANK_R, L.BODY_TOP - .006),
                                              (L.TANK_R - .003, L.BODY_TOP + .004), (L.NECK_R + .003, L.NECK_Z - .004), (L.NECK_R, L.NECK_Z)]))
    level = L.WATER_L * 1e-3 / (math.pi * (L.TANK_R - .002) ** 2)
    parts.append(lathe("water", "water", [(L.TANK_R - .003, .003), (L.TANK_R - .003, level)]))
    parts.append(lathe("label", "label", [(L.TANK_R + .0004, .060), (L.TANK_R + .0004, .125)]))
    mark_r = L.NECK_R + .006
    parts.append(hull("neck_mark", "mark_white", swept(math.pi, [(mark_r - .004, L.shoulder_z(mark_r - .004) + .0008), (mark_r + .006, L.shoulder_z(mark_r + .006) + .0008),
                                                                  (mark_r + .006, L.shoulder_z(mark_r + .006)), (mark_r - .004, L.shoulder_z(mark_r - .004))], .004), collide=False))
    # a thinner tick 10 degrees round from it: the collar's mark passing it is the hazard tolerance
    parts.append(hull("neck_tolerance_tick", "mark_white", swept(math.pi + L.SEAL_OPEN, [(mark_r - .004, L.shoulder_z(mark_r - .004) + .0008), (mark_r + .006, L.shoulder_z(mark_r + .006) + .0008),
                                                                  (mark_r + .006, L.shoulder_z(mark_r + .006)), (mark_r - .004, L.shoulder_z(mark_r - .004))], .0025), collide=False))
    # the valve's seat (drawn only)
    seat = frame(L.valve_axis())
    parts.append(lathe("valve_seat", "head_black", [(.008, -.003), (.008, .002)], rotation=seat, origin=L.valve_base(), segments=16))
    return "".join(parts)


def head_parts():
    """In the head's frame: origin on the axis at the top of the tank's neck."""
    def drop(points):
        return np.asarray(points, dtype=float) - (0., 0., L.NECK_Z)

    a, b = L.NOZZLE_START, L.NOZZLE_START + math.pi     # nozzle side, handle side
    half = L.NOZZLE_W / 2
    parts = [
        hull("collar_collider", "head_black", drop(ring(L.NECK_R, L.NECK_Z) + ring(L.COLLAR_TOP_R, L.COLLAR_TOP)), visible=False),
        hull("top_collider", "head_black", drop(ring(L.HEAD_R, L.COLLAR_TOP) + ring(L.HEAD_R, L.HEAD_TOP)), visible=False),
        hull("nozzle_collider", "brass", drop(bar(a, .028, L.NOZZLE_TIP_R, L.NOZZLE_W, L.NOZZLE_Z - half, L.NOZZLE_Z + half)), visible=False, offset=.0005),
        hull("handle_root", "head_black", drop(bar(b, .028, .062, L.HANDLE_W, .256, .270))),
        hull("handle", "head_black", drop(swept(b, [(.046, .270), (.064, .270), (.084, L.HANDLE_BOTTOM), (.066, L.HANDLE_BOTTOM)], L.HANDLE_W))),
        hull("trigger", "trigger_red", drop(bar(b, .034, .062, .014, .272, .282))),
        lathe("collar", "head_black", [(L.NECK_R, 0.), (L.COLLAR_TOP_R, L.COLLAR_TOP - L.NECK_Z), (L.HEAD_R, L.COLLAR_TOP - L.NECK_Z),
                                       (L.HEAD_R, L.HEAD_TOP - L.NECK_Z - .003), (L.HEAD_R - .004, L.HEAD_TOP - L.NECK_Z)]),
    ]
    for k in range(18):                                 # grip ribs round the collar (drawn only)
        az = 2 * math.pi * k / 18
        r0 = L.NECK_R - .001
        parts.append(hull(f"rib_{k:02d}", "head_black", drop(swept(az, [(r0, L.NECK_Z + .003), (r0 + .003, L.NECK_Z + .003),
                                                                         (L.COLLAR_TOP_R + .0015, L.COLLAR_TOP - .004), (L.COLLAR_TOP_R - .002, L.COLLAR_TOP - .004)], .004)), collide=False))
    # the spout (black) and the brass nozzle on its end, drawn round
    along = frame((math.cos(a), math.sin(a), 0.))
    at = np.array((0., 0., L.NOZZLE_Z - L.NECK_Z))
    parts.append(lathe("spout", "head_black", [(half, .028), (half, .058)], rotation=along, origin=at, segments=16))
    parts.append(lathe("nozzle", "brass", [(half - .0005, .058), (half + .0005, .060), (half + .0005, .072), (half - .001, L.NOZZLE_TIP_R)],
                       rotation=along, origin=at, segments=16))
    # the collar's white mark, in line with the tank's at the start
    parts.append(hull("collar_mark", "mark_white", drop(swept(math.pi, [(L.NECK_R + .0005, L.NECK_Z + .004), (L.NECK_R + .0035, L.NECK_Z + .004),
                                                                         (L.COLLAR_TOP_R + .002, L.NECK_Z + .016), (L.COLLAR_TOP_R - .001, L.NECK_Z + .016)], .005)), collide=False))
    return "".join(parts)


def knob_parts():
    """In the knob's frame: origin on the axis at the top of the head."""
    z0, z1, top = 0., L.KNOB_BOTTOM - L.HEAD_TOP, L.HEIGHT - L.HEAD_TOP
    return (hull("stem_collider", "steel", ring(L.STEM_R, z0, 12) + ring(L.STEM_R, z1, 12), visible=False)
            + hull("knob_collider", "head_black", ring(L.KNOB_R, z1) + ring(L.KNOB_R, top), visible=False)
            + lathe("stem", "steel", [(L.STEM_R, z0), (L.STEM_R, z1)], segments=16)
            + lathe("knob", "head_black", [(L.STEM_R + .004, z1 - .004), (L.KNOB_R - .002, z1), (L.KNOB_R, z1 + .004), (L.KNOB_R, top - .004), (L.KNOB_R - .004, top)]))


def valve_parts():
    """In the valve's frame: origin at the middle of the cap, +z out along the valve."""
    h, l = L.VALVE_CAP / 2, L.VALVE_CAP_LEN / 2
    box = [(sx * h, sy * h, sz * l) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return (hull("cap_collider", "valve_red", box, visible=False, offset=.0005)
            + lathe("cap", "valve_red", [(h - .001, -l), (h, -l + .002), (h, l - .002), (h - .002, l)], segments=16)
            + lathe("valve_stem", "steel", [(.003, -l - L.VALVE_STEM - L.VALVE_TRAVEL), (.003, -l)], segments=8))


def usd():
    top = scene.TOP

    def place(point):
        p = scene.to_station(point)
        return (p[0], p[1], p[2] + top)

    mx, my, mz = L.MAT
    cx, cy = scene.MARK
    mat = ('    def Xform "Mat"\n    {\n' + "\n".join("    " + line for line in _static_box(
        "mat", "mat_rubber", (cx - mx / 2, cy - my / 2, top), (cx + mx / 2, cy + my / 2, top + mz)).splitlines()) + "\n    }\n\n")

    bodies = body("tank", L.TANK_MASS, place((0, 0, 0)), tank_parts(), root=True, com=(0., 0., L.TANK_COM_Z))
    bodies += body("head", L.HEAD_MASS, place((0, 0, L.NECK_Z)), head_parts())
    bodies += body("knob", L.KNOB_MASS, place((0, 0, L.HEAD_TOP)), knob_parts())
    v = frame(L.valve_axis())
    bodies += body("valve", L.VALVE_MASS, place(L.valve_cap_centre()), valve_parts(), orient=wxyz(v), inertia=(2e-7, 2e-7, 2e-7))

    f0, f1 = L.VALVE_SPRING
    rate = (f1 - f0) / L.VALVE_TRAVEL
    joints = hinge_z("head", "tank", "head", (0., 0., L.NECK_Z), limits=(0., math.degrees(L.HEAD_FREE)), drag=L.HEAD_DRAG, armature=.0002)
    joints += hinge_z("knob", "head", "knob", (0., 0., L.HEAD_TOP - L.NECK_Z), damping=1e-6, armature=.00002)
    joints += slide_z("valve", "tank", "valve", L.valve_cap_centre(), wxyz(v), L.VALVE_TRAVEL, stiffness=rate, target=-f0 / rate,
                      damping=4., armature=.02)
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n' + mat + '    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


def _static_box(name, material_name, lo, hi):
    """A fixed box collider (no rigid body: it does not move), drawn."""
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    centre, half = (lo + hi) / 2, (hi - lo) / 2
    return (f'    def Cube "{name}" (\n        prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]\n    )\n    {{\n'
            f"        rel material:binding = {_PREFIX}{material_name}>\n"
            f"        rel material:binding:physics = {_PREFIX}{material_name}>\n"
            "        float physxCollision:contactOffset = 0.001\n        float physxCollision:restOffset = 0\n"
            "        double size = 2\n"
            f"        float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
            f"        double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            "    }\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print("wrote", scene.USD.name, f"{scene.USD.stat().st_size / 1e3:.0f} kB")
