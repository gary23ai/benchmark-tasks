"""Regenerate ``rdb_child_latch/drawer.usda`` from ``layout.py``.

Run from the task folder: ``python tools/build_scene.py``. Needs only NumPy.

The frame and the latch's catch are fixed boxes. The drawer is one rigid body on a slide along x whose
friction is a force-capped damper. The latch arm is a rigid body on a pin in the drawer, held up against its
stop by a torsion spring (an angular drive with a target past the stop); it is free to be pressed down 10
degrees. Every part is a box with full collision. The latch does not collide with the drawer it is screwed to.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_child_latch import fixed_scene as scene  # noqa: E402
from rdb_child_latch import layout as L  # noqa: E402

# colour (linear RGB), roughness, opacity, friction key
MATERIALS = {
    "frame": ((.93, .93, .91), .35, 1., "frame"),
    "drawer": ((.90, .93, .95), .08, .22, "drawer"),
    "latch": ((.97, .97, .95), .45, 1., "latch"),
    "pull": ((.90, .93, .95), .08, .22, "pull"),
}
_PREFIX = "</Fixture/Materials/"
DRAWER_DAMPING = 30000.          # N s/m: with the force cap this is dry friction down to 0.1 mm/s
SLIDE_TO_MINUS_X = (0., 0., .70710677, .70710677)     # (w, x, y, z): the joint's axis is -x, so its value is "open"
RAMP_TURN = (float(np.cos(np.pi / 8)), 0., float(np.sin(np.pi / 8)), 0.)   # 45 degrees about y
DEG = np.pi / 180.


def box(name, material, low, high, indent=12, orient=None, centre=None, half=None):
    """A colliding box given by two opposite corners in its parent's frame (or centre, half sizes, orient)."""
    pad = " " * indent
    if centre is None:
        centre = [(a + b) / 2 for a, b in zip(low, high)]
        half = [(b - a) / 2 for a, b in zip(low, high)]
    ops = '"xformOp:translate", "xformOp:scale"'
    rotation = ""
    if orient is not None:
        ops = '"xformOp:translate", "xformOp:orient", "xformOp:scale"'
        rotation = f"{pad}    quatf xformOp:orient = ({orient[0]:.9g}, {orient[1]:.9g}, {orient[2]:.9g}, {orient[3]:.9g})\n"
    return (f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", '
            f'"PhysxCollisionAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {_PREFIX}{material}>\n"
            f"{pad}    rel material:binding:physics = {_PREFIX}{material}>\n"
            f"{pad}    float physxCollision:contactOffset = 0.001\n"
            f"{pad}    float physxCollision:restOffset = 0\n"
            f"{pad}    double size = 2\n"
            f"{pad}    float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
            f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
            + rotation +
            f"{pad}    uniform token[] xformOpOrder = [{ops}]\n"
            f"{pad}}}\n\n")


def body(name, mass, position, children, no_contact_with=()):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    pairs = ""
    if no_contact_with:
        apis.append('"PhysicsFilteredPairsAPI"')
        pairs = ("            rel physics:filteredPairs = [\n"
                 + "".join(f"                </Fixture/Actors/{other}>,\n" for other in no_contact_with)
                 + "            ]\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            + pairs +
            f"            float physics:mass = {mass:.6g}\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            "            quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"            double3 xformOp:translate = ({position[0]:.6g}, {position[1]:.6g}, {position[2]:.6g})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def slide(name, body1, anchor0, rotation, low, high, *, damping, max_force):
    """The drawer's slide, from the world. ``anchor0`` is in the world frame; body1's origin is on it at 0."""
    w, x, y, z = rotation
    return (f'        def PhysicsPrismaticJoint "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:linear"]\n'
            "        )\n        {\n"
            f"            float drive:linear:physics:damping = {damping:.6g}\n"
            f"            float drive:linear:physics:maxForce = {max_force:.6g}\n"
            "            float drive:linear:physics:stiffness = 0\n"
            "            float drive:linear:physics:targetPosition = 0\n"
            "            float drive:linear:physics:targetVelocity = 0\n"
            '            uniform token drive:linear:physics:type = "force"\n'
            '            uniform token physics:axis = "X"\n'
            f"            rel physics:body1 = </Fixture/Actors/{body1}>\n"
            "            bool physics:collisionEnabled = 0\n"
            "            uniform bool physics:excludeFromArticulation = 1\n"
            f"            point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            f"            quatf physics:localRot0 = ({w:.9g}, {x:.9g}, {y:.9g}, {z:.9g})\n"
            f"            quatf physics:localRot1 = ({w:.9g}, {x:.9g}, {y:.9g}, {z:.9g})\n"
            f"            float physics:lowerLimit = {low:.6g}\n"
            f"            float physics:upperLimit = {high:.6g}\n"
            "        }\n\n")


def pin():
    """The latch arm's pin: a revolute joint about y in the drawer. A positive angle presses the arm's far
    (hook) end down. USD angular drives and limits are in degrees."""
    anchor = (L.HINGE_X, L.LATCH_Y, L.HINGE_Z)
    return ('        def PhysicsRevoluteJoint "latch_pin" (\n            prepend apiSchemas = ["PhysicsDriveAPI:angular"]\n'
            "        )\n        {\n"
            f"            float drive:angular:physics:damping = {L.LATCH_DAMPING * DEG:.6g}\n"
            f"            float drive:angular:physics:stiffness = {L.LATCH_STIFFNESS * DEG:.6g}\n"
            f"            float drive:angular:physics:targetPosition = {-np.degrees(L.LATCH_PRELOAD):.6g}\n"
            "            float drive:angular:physics:targetVelocity = 0\n"
            '            uniform token drive:angular:physics:type = "force"\n'
            '            uniform token physics:axis = "Y"\n'
            "            rel physics:body0 = </Fixture/Actors/drawer>\n"
            "            rel physics:body1 = </Fixture/Actors/latch>\n"
            "            bool physics:collisionEnabled = 0\n"
            "            uniform bool physics:excludeFromArticulation = 1\n"
            f"            point3f physics:localPos0 = ({anchor[0]:.6g}, {anchor[1]:.6g}, {anchor[2]:.6g})\n"
            "            point3f physics:localPos1 = (0, 0, 0)\n"
            "            quatf physics:localRot0 = (1, 0, 0, 0)\n"
            "            quatf physics:localRot1 = (1, 0, 0, 0)\n"
            "            float physics:lowerLimit = 0\n"
            f"            float physics:upperLimit = {np.degrees(L.LATCH_TRAVEL):.6g}\n"
            "        }\n\n")


def material(name):
    color, roughness, opacity, key = MATERIALS[name]
    friction = L.FRICTION[key]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:opacity = {opacity}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def frame_boxes():
    """(name, low corner, high corner) of the frame's fixed boxes, tabletop frame."""
    x0, x1 = L.FRAME_FRONT_X, L.FRAME_FRONT_X + L.FRAME_SIZE[0]
    half, inner = L.FRAME_SIZE[1] / 2, L.FRAME_SIZE[1] / 2 - L.FRAME_WALL
    return [("top", (x0, -half, L.RAIL_Z), (x1, half, L.FRAME_SIZE[2])),
            ("side_left", (x0, inner, 0.), (x1, half, L.RAIL_Z)),
            ("side_right", (x0, -half, 0.), (x1, -inner, L.RAIL_Z)),
            ("back", (x1 - L.FRAME_BACK, -inner, 0.), (x1, inner, L.RAIL_Z)),
            ("base", (x0, -inner, 0.), (x1, inner, .010))]


def catch_box():
    """The catch under the frame's top, tabletop frame."""
    half = L.CATCH_WIDTH / 2
    return ("catch", (L.CATCH_FACE_X - L.CATCH_DEPTH, L.LATCH_Y - half, L.CATCH_LOW_Z),
            (L.CATCH_FACE_X, L.LATCH_Y + half, L.RAIL_Z))


def drawer_boxes():
    """(name, low, high) of the drawer's boxes in the drawer's frame (origin: outer face of its front, on the
    table plane). The latch's base plate is screwed to the drawer front, so it is part of the drawer."""
    inner_x0, inner_x1 = L.FRONT_T, L.FRONT_T + L.DRAWER_INNER[0]
    inner, outer = L.DRAWER_INNER[1] / 2, L.DRAWER_INNER[1] / 2 + L.DRAWER_WALL
    floor0 = L.FLOOR_Z - L.DRAWER_FLOOR_T
    px, py, pz = L.PULL_SIZE
    w = L.LATCH_WIDTH / 2
    return [("front", (0., -L.FRONT_WIDTH / 2, L.FRONT_Z[0]), (L.FRONT_T, L.FRONT_WIDTH / 2, L.FRONT_Z[1])),
            ("pull", (-px, -py / 2, L.PULL_Z - pz / 2), (0., py / 2, L.PULL_Z + pz / 2)),
            ("floor", (inner_x0, -outer, floor0), (inner_x1 + L.DRAWER_BACK, outer, L.FLOOR_Z)),
            ("wall_left", (inner_x0, inner, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, outer, L.RIM_Z)),
            ("wall_right", (inner_x0, -outer, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, -inner, L.RIM_Z)),
            ("wall_back", (inner_x1, -inner, L.FLOOR_Z), (inner_x1 + L.DRAWER_BACK, inner, L.RIM_Z)),
            ("latch_base", (L.FRONT_T, L.LATCH_Y - w, L.PLATE_Z[0]), (L.HINGE_X, L.LATCH_Y + w, L.PLATE_Z[1]))]


def latch_boxes():
    """(name, low, high) of the latch arm's boxes in its own frame (origin on the pin, arm at rest along +x)."""
    w = L.LATCH_WIDTH / 2
    end = L.FRONT_T + L.LATCH_LENGTH - L.HINGE_X
    top = L.ARM_TOP_Z - L.HINGE_Z
    return [("arm", (0., -w, top - L.ARM_T), (end, w, top)),
            ("hook", (L.HOOK_FACE_X - L.HINGE_X, -w, top), (end, w, top + L.HOOK_RISE))]


def ramp():
    """The hook's rounded back: centre and half sizes (latch frame) of a box turned 45 degrees about y."""
    centre = (L.FRONT_T + L.LATCH_LENGTH - L.HINGE_X, 0., L.ARM_TOP_Z - L.HINGE_Z)
    return centre, (L.RAMP_HALF, L.LATCH_WIDTH / 2, L.RAMP_HALF)


def usd():
    top = scene.TOP
    start = scene.WORLD_START
    bodies = body("drawer", L.DRAWER_MASS, start["drawer"],
                  "".join(box(name, {"latch_base": "latch", "pull": "pull"}.get(name, "drawer"), low, high)
                          for name, low, high in drawer_boxes()))
    centre, half = ramp()
    bodies += body("latch", L.LATCH_MASS, start["latch"],
                   "".join(box(name, "latch", low, high) for name, low, high in latch_boxes())
                   + box("ramp", "latch", None, None, orient=RAMP_TURN, centre=centre, half=half),
                   no_contact_with=("drawer",))
    name, low, high = catch_box()
    fixed = "".join(box(n, "frame", (lo[0], lo[1], lo[2] + top), (hi[0], hi[1], hi[2] + top), indent=8)
                    for n, lo, hi in frame_boxes())
    fixed += box(name, "latch", (low[0], low[1], low[2] + top), (high[0], high[1], high[2] + top), indent=8)
    joints = slide("drawer_slide", "drawer", (L.FRAME_FRONT_X, 0., top), SLIDE_TO_MINUS_X, 0., L.OPEN_LIMIT,
                   damping=DRAWER_DAMPING, max_force=L.RUNNER_FRICTION_N)
    joints += pin()
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Scope "Static"\n    {\n' + fixed + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD.name}: {len(L.ACTORS)} bodies, a slide and a pin, {len(frame_boxes()) + 1} fixed boxes; "
          f"the hook meets the catch at an opening of {1000 * L.catch_open():.1f} mm and clears it past "
          f"{1000 * L.hook_clear_open():.1f} mm; it overlaps the catch by {1000 * L.ENGAGE:.1f} mm and is free "
          f"once the arm is pressed {np.degrees(L.RELEASE_ANGLE):.1f} degrees")
