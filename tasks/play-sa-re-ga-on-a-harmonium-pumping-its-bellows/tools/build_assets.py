"""Regenerate ``rdb_harmonium/harmonium.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs only NumPy.

The harmonium is one free articulation of 41 bodies: the case (root, on four rubber feet on the table), the
bellows board at the back on a revolute joint along its bottom edge, and all 39 keys on vertical sprung slides.
There are no loop joints. Collision shapes are plain boxes and are not rendered; what the cameras see are display
meshes with no collision: the wooden case with its stop rail and seven round stop knobs, the white and black keys
(the three marked keys carry a coloured dot), and the bellows board. The leather folds between the board and the
case change shape as the board moves, so the adapter draws them at run time (``skin.py``).
"""

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_harmonium import fixed_scene as scene  # noqa: E402
from rdb_harmonium import layout as L  # noqa: E402

# colour, roughness, metallic, friction
MATERIALS = {
    "wood": ((.36, .19, .09), .55, 0., .50),          # the teak-stained case
    "dark_wood": ((.20, .10, .05), .5, 0., .50),
    "rubber": ((.05, .05, .05), .9, 0., .80),          # the feet on the wooden table
    "ivory": ((.92, .90, .84), .35, 0., .60),          # white keys
    "ebony": ((.04, .04, .04), .3, 0., .60),           # black keys
    "knob": ((.88, .83, .66), .4, 0., .50),            # cream stop knobs
    "board": ((.40, .22, .11), .55, 0., .90),          # the bellows board (wood under a rubber gripper pad)
    "dot_C4": (L.DOT_COLOURS["C4"], .5, 0., .60),
    "dot_D4": (L.DOT_COLOURS["D4"], .5, 0., .60),
    "dot_E4": (L.DOT_COLOURS["E4"], .5, 0., .60),
}
_PREFIX = "</Fixture/Materials/"
FLAP_MASS = L.FLAP_MASS
KEY_MASSES = [L.key_box(m)[6] for m in L.KEY_MIDI]
CASE_MASS = L.MASS - FLAP_MASS - sum(KEY_MASSES)


def _f(v):
    return f"{v:.6g}"


def _points(points):
    return "[" + ", ".join(f"({_f(x)}, {_f(y)}, {_f(z)})" for x, y, z in points) + "]"


def box(name, material, lo, hi):
    """An unrendered box collider from corner ``lo`` to corner ``hi`` (body coordinates)."""
    c = [(a + b) / 2 for a, b in zip(lo, hi, strict=True)]
    h = [(b - a) / 2 for a, b in zip(lo, hi, strict=True)]
    return (f'            def Cube "{name}" (\n'
            '                prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
            "            )\n            {\n"
            f"                rel material:binding = {_PREFIX}{material}>\n"
            f"                rel material:binding:physics = {_PREFIX}{material}>\n"
            "                float physxCollision:contactOffset = 0.002\n"
            "                float physxCollision:restOffset = 0\n"
            '                token visibility = "invisible"\n'
            "                double size = 2\n"
            f"                float3 xformOp:scale = ({_f(h[0])}, {_f(h[1])}, {_f(h[2])})\n"
            f"                double3 xformOp:translate = ({_f(c[0])}, {_f(c[1])}, {_f(c[2])})\n"
            '                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            "            }\n\n")


def mesh(name, material, points, faces):
    """A rendered mesh with no collision."""
    return (f'            def Mesh "{name}" (\n                prepend apiSchemas = ["MaterialBindingAPI"]\n            )\n'
            "            {\n"
            f"                rel material:binding = {_PREFIX}{material}>\n"
            "                uniform bool doubleSided = 1\n"
            f"                int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"                int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"                point3f[] points = {_points(points)}\n"
            '                uniform token subdivisionScheme = "none"\n'
            "            }\n\n")


def box_mesh(name, material, lo, hi, bevel=0.):
    """A rendered box (optionally with its top edges bevelled by ``bevel``)."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    b = bevel
    points = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
              (x0, y0, z1 - b), (x1, y0, z1 - b), (x1, y1, z1 - b), (x0, y1, z1 - b),
              (x0 + b, y0 + b, z1), (x1 - b, y0 + b, z1), (x1 - b, y1 - b, z1), (x0 + b, y1 - b, z1)]
    faces = [[3, 2, 1, 0], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
             [4, 5, 9, 8], [5, 6, 10, 9], [6, 7, 11, 10], [7, 4, 8, 11], [8, 9, 10, 11]]
    return mesh(name, material, points, faces)


def cylinder_mesh(name, material, centre, radius, length, axis="x", segments=20):
    """A rendered cylinder along ``axis`` starting at ``centre`` and running ``length`` (negative: backwards)."""
    cx, cy, cz = centre
    ring = [(radius * math.cos(2 * math.pi * k / segments), radius * math.sin(2 * math.pi * k / segments))
            for k in range(segments)]

    def at(u, v, t):
        if axis == "x":
            return (cx + t, cy + u, cz + v)
        return (cx + u, cy + v, cz + t)

    points = [at(u, v, 0.) for u, v in ring] + [at(u, v, length) for u, v in ring] + [at(0, 0, 0.), at(0, 0, length)]
    n = segments
    faces = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    faces += [[(i + 1) % n, i, 2 * n] for i in range(n)] + [[n + i, n + (i + 1) % n, 2 * n + 1] for i in range(n)]
    return mesh(name, material, points, faces)


def disc_mesh(name, material, centre, radius, segments=16):
    cx, cy, cz = centre
    points = [(cx + radius * math.cos(2 * math.pi * k / segments), cy + radius * math.sin(2 * math.pi * k / segments),
               cz) for k in range(segments)]
    return mesh(name, material, points, [list(range(segments))])


# ---------------------------------------------------------------------------------------------------------

def case_parts():
    hw = L.CASE_W / 2
    kl = L.KEYBOARD_LEFT_Y
    rail0, rail1, rail_top = L.STOP_RAIL
    feet = []
    for i, (fx, fy) in enumerate(((.03, hw - .03), (.03, -hw + .03), (L.BACK_X - .03, hw - .03),
                                  (L.BACK_X - .03, -hw + .03))):
        feet.append(box(f"foot_{i}", "rubber", (fx - .015, fy - .015, 0.), (fx + .015, fy + .015, L.FEET_H)))
    colliders = "".join(feet) + "".join((
        box("body", "wood", (0., -hw, L.FEET_H), (L.BACK_X, hw, L.KEYBED_Z)),
        box("top", "wood", (rail0, -hw, L.KEYBED_Z), (L.BACK_X, hw, L.CASE_H)),
        box("cheek_left", "wood", (0., kl, L.KEYBED_Z), (rail0, hw, L.CASE_H - .012)),
        box("cheek_right", "wood", (0., -hw, L.KEYBED_Z), (rail0, -kl, L.CASE_H - .012)),
    ))
    looks = "".join((
        box_mesh("body_look", "wood", (0., -hw, L.FEET_H), (L.BACK_X, hw, L.KEYBED_Z)),
        box_mesh("front_band_look", "dark_wood", (-.002, -hw + .01, L.FEET_H + .02), (0., hw - .01, L.KEYBED_Z - .015)),
        box_mesh("rail_look", "dark_wood", (rail0, -hw, L.KEYBED_Z), (rail1, hw, rail_top), bevel=.003),
        box_mesh("top_look", "wood", (rail1, -hw, L.KEYBED_Z), (L.BACK_X, hw, L.CASE_H), bevel=.004),
        box_mesh("cheek_left_look", "wood", (0., kl, L.KEYBED_Z), (rail0, hw, L.CASE_H - .012), bevel=.004),
        box_mesh("cheek_right_look", "wood", (0., -hw, L.KEYBED_Z), (rail0, -kl, L.CASE_H - .012), bevel=.004),
    ))
    for i, (fx, fy) in enumerate(((.03, hw - .03), (.03, -hw + .03), (L.BACK_X - .03, hw - .03),
                                  (L.BACK_X - .03, -hw + .03))):
        looks += box_mesh(f"foot_{i}_look", "rubber", (fx - .015, fy - .015, 0.), (fx + .015, fy + .015, L.FEET_H))
    # seven stop knobs on the rail's front face; the main stop (second from the player's left) pulled out
    spacing = (2 * kl - .06) / (L.STOPS - 1)
    for i in range(L.STOPS):
        y = kl - .03 - i * spacing
        pull = L.MAIN_STOP_PULL if i == 1 else 0.
        looks += cylinder_mesh(f"stop_{i}_shaft", "dark_wood", (rail0, y, L.STOP_KNOB_Z), .004, -.006 - pull)
        looks += cylinder_mesh(f"stop_{i}_knob", "knob", (rail0 - .006 - pull, y, L.STOP_KNOB_Z),
                               L.STOP_KNOB_D / 2, -L.STOP_KNOB_L)
    return colliders + looks


def key_parts(midi):
    """A key's collider and look, in its own body frame (origin at the middle of its top face at rest)."""
    x, y, top, sx, sy, t, _, white = L.key_box(midi)
    name = L.key_name(midi)
    lo, hi = (-sx / 2, -sy / 2, -t), (sx / 2, sy / 2, 0.)
    out = box("collider", "ivory" if white else "ebony", lo, hi)
    out += box_mesh("look", "ivory" if white else "ebony", lo, hi, bevel=.0008 if white else .0015)
    if name in L.DOT_COLOURS:
        out += disc_mesh("dot", f"dot_{name}", (-sx / 2 + L.DOT_FROM_FRONT, 0., .0003), .0045)
    return out


def flap_parts():
    lo, hi = (0., -L.FLAP_W / 2, 0.), (L.FLAP_T, L.FLAP_W / 2, L.FLAP_H)
    return box("collider", "board", lo, hi) + box_mesh("look", "board", lo, hi, bevel=.002)


def body(name, mass, position, children, *, root=False, pitch=0.):
    """A rigid body at ``position``, turned ``pitch`` rad about +y (the bellows board is authored open)."""
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = ("            bool physxArticulation:enabledSelfCollisions = 0\n"
                 "            int physxArticulation:solverPositionIterationCount = 32\n"
                 "            int physxArticulation:solverVelocityIterationCount = 8\n"
                 "            float physxArticulation:sleepThreshold = 0\n"
                 "            float physxArticulation:stabilizationThreshold = 0\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {_f(mass)}\n"
            "            bool physics:rigidBodyEnabled = 1\n" + extra +
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"            quatf xformOp:orient = ({math.cos(pitch / 2):.7g}, 0, {math.sin(pitch / 2):.7g}, 0)\n"
            f"            double3 xformOp:translate = ({_f(position[0])}, {_f(position[1])}, {_f(position[2])})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def flap_joint(anchor):
    """The bellows board's hinge along y. Its drive carries the board's torque: a velocity drive toward opening
    whose torque is capped at FLAP_TORQUE_NM, so the board is held open and takes that torque to push shut."""
    deg = math.pi / 180
    return (f'        def PhysicsRevoluteJoint "{scene.FLAP_JOINT}" (\n'
            '            prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI"]\n        )\n        {\n'
            f"            float drive:angular:physics:damping = {_f(L.FLAP_DAMPING * deg)}\n"
            f"            float drive:angular:physics:maxForce = {_f(L.FLAP_TORQUE_NM)}\n"
            "            float drive:angular:physics:stiffness = 0\n"
            "            float drive:angular:physics:targetPosition = 0\n"
            f"            float drive:angular:physics:targetVelocity = {_f(math.degrees(L.FLAP_OPEN_SPEED))}\n"
            '            uniform token drive:angular:physics:type = "force"\n'
            '            uniform token physics:axis = "Y"\n'
            "            float physics:lowerLimit = 0\n"
            f"            float physics:upperLimit = {_f(math.degrees(L.FLAP_OPEN))}\n"
            "            rel physics:body0 = </Fixture/Actors/case>\n"
            "            rel physics:body1 = </Fixture/Actors/flap>\n"
            f"            point3f physics:localPos0 = ({_f(anchor[0])}, {_f(anchor[1])}, {_f(anchor[2])})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            "            quatf physics:localRot0 = (1, 0, 0, 0)\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n"
            "            float physxJoint:armature = 0.002\n"
            "            float physxJoint:maxJointVelocity = 360\n"
            "        }\n\n")


def key_joint(name, prim, anchor):
    """A key's vertical slide: 7 mm down to its stop, sprung up against its top stop with a preload."""
    return (f'        def PhysicsPrismaticJoint "{name}" (\n'
            '            prepend apiSchemas = ["PhysicsDriveAPI:linear", "PhysxJointAPI"]\n        )\n        {\n'
            "            float drive:linear:physics:damping = 1\n"
            "            float drive:linear:physics:maxForce = 50\n"
            f"            float drive:linear:physics:stiffness = {_f(L.KEY_RATE)}\n"
            f"            float drive:linear:physics:targetPosition = {_f(L.KEY_PRELOAD)}\n"
            '            uniform token drive:linear:physics:type = "force"\n'
            '            uniform token physics:axis = "Z"\n'
            f"            float physics:lowerLimit = {_f(-L.KEY_TRAVEL)}\n"
            "            float physics:upperLimit = 0\n"
            "            rel physics:body0 = </Fixture/Actors/case>\n"
            f"            rel physics:body1 = </Fixture/Actors/{prim}>\n"
            f"            point3f physics:localPos0 = ({_f(anchor[0])}, {_f(anchor[1])}, {_f(anchor[2])})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            "            quatf physics:localRot0 = (1, 0, 0, 0)\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n"
            "            float physxJoint:armature = 0.01\n"
            "        }\n\n")


def material(name):
    color, roughness, metallic, friction = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]:.4g}, {color[1]:.4g}, {color[2]:.4g})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def usd():
    top = scene.TOP
    origin = np.asarray(scene.CASE_START) + (0., 0., top)
    bodies = body("case", CASE_MASS, origin, case_parts(), root=True)
    hinge = (L.HINGE[0], 0., L.HINGE[1])
    bodies += body("flap", FLAP_MASS, origin + hinge, flap_parts(), pitch=L.FLAP_OPEN)   # authored open, at rest
    joints = flap_joint(hinge)
    for midi, link, joint in zip(L.KEY_MIDI, scene.LINKS[2:], scene.KEY_JOINTS, strict=True):
        x, y, top_z, *_ = L.key_box(midi)
        prim = link
        bodies += body(prim, L.key_box(midi)[6], origin + (x, y, top_z), key_parts(midi))
        joints += key_joint(joint, prim, (x, y, top_z))
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    text = usd()
    (ROOT / "rdb_harmonium/harmonium.usda").write_text(text)
    print("written", len(text), "characters")
