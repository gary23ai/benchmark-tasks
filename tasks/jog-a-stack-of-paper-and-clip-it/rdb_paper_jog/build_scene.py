"""Regenerate ``rdb_paper_jog/paper_jog.usda`` from ``paper_model.py``:  python -m rdb_paper_jog.build_scene

Loose bodies (no articulation):

- ``sheet_0`` .. ``sheet_8``: the stack, nine rigid plates of four sheets each, one box collider per plate, free
  to slide on one another;
- ``clip_a``, ``clip_b``: the two halves of the binder clip (jaw, rolled lip, half of the back, wire handle),
  joined by one hinge along the clip's back with the spring between them. Squeezing the two handles together
  turns the halves about the hinge and opens the lips; nothing else opens the clip.

Fixed colliders: the two stacked reams the stack starts on (one box) and the fin the clip is parked on. The fin's foot is drawn
only. Nothing is marked on the table.
"""

import math

import numpy as np

from . import fixed_scene as scene
from . import paper_model as m

# name: colour (linear RGB), roughness, metallic, friction key (None: display only)
MATERIALS = {
    "paper_a": ((.86, .86, .83), .9, 0., "paper"), "paper_b": ((.70, .70, .68), .9, 0., "paper"),
    "ink": ((.10, .10, .12), .9, 0., None),
    "clip_black": ((.02, .02, .025), .35, .6, "clip"), "wire": ((.75, .76, .78), .25, 1., "clip"),
    "mdf": ((.42, .30, .19), .8, 0., "block"),
    "wrapper": ((.80, .82, .86), .55, 0., "block"), "wrapper_band": ((.10, .28, .55), .55, 0., None),
    "acrylic": ((.55, .70, .80), .2, 0., "acrylic"),
}
_M = "</Fixture/Materials/"


def _fmt(v):
    return ", ".join(f"{float(c):.7g}" for c in v)


def wxyz(rotation):
    x, y, z, w = m.quat_xyzw(rotation)
    return (w, x, y, z)


def _prim(kind, name, material, centre, *, collide, scale=None, orient=None, extra="", indent=12):
    pad = " " * indent
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    ops = ['"xformOp:translate"'] + (['"xformOp:orient"'] if orient is not None else []) + \
          (['"xformOp:scale"'] if scale is not None else [])
    text = (f'{pad}def {kind} "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {_M}{material}>\n")
    if collide:
        text += (f"{pad}    rel material:binding:physics = {_M}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = {m.CONTACT_OFFSET:.6g}\n"
                 f"{pad}    float physxCollision:restOffset = 0\n")
    text += "".join(f"{pad}    {line}\n" for line in extra.splitlines())
    text += f"{pad}    double3 xformOp:translate = ({_fmt(centre)})\n"
    if orient is not None:
        text += f"{pad}    quatf xformOp:orient = ({_fmt(orient)})\n"
    if scale is not None:
        text += f"{pad}    float3 xformOp:scale = ({_fmt(scale)})\n"
    return text + f"{pad}    uniform token[] xformOpOrder = [{', '.join(ops)}]\n{pad}}}\n\n"


def box(name, material, centre, size, *, collide=True, turn_y=None, **kw):
    """A box of full ``size`` about ``centre``, optionally turned about its own y axis by ``turn_y`` radians."""
    orient = wxyz(m.rot_y(turn_y)) if turn_y is not None else None
    return _prim("Cube", name, material, centre, collide=collide, scale=np.asarray(size, float) / 2, orient=orient,
                 extra="double size = 2", **kw)


def capsule(name, material, centre, radius, height, axis="Y", **kw):
    return _prim("Capsule", name, material, centre, collide=True,
                 extra=f'uniform token axis = "{axis}"\ndouble height = {height:.6g}\ndouble radius = {radius:.6g}', **kw)


def body(name, mass, position, rotation, children, no_contact_with=()):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    pairs = ""
    if no_contact_with:
        apis.append('"PhysicsFilteredPairsAPI"')
        pairs = ("            rel physics:filteredPairs = [\n"
                 + ",\n".join(f"                </Fixture/Actors/{other}>" for other in no_contact_with) + "\n            ]\n")
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            + pairs +
            f"            float physics:mass = {mass:.6g}\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxContactReport:threshold = 0\n"
            f"            float physxRigidBody:maxDepenetrationVelocity = {m.MAX_DEPENETRATION:.6g}\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            f"            int physxRigidBody:solverPositionIterationCount = {m.SOLVER_ITERATIONS[0]}\n"
            f"            int physxRigidBody:solverVelocityIterationCount = {m.SOLVER_ITERATIONS[1]}\n"
            f"            quatf xformOp:orient = ({_fmt(wxyz(rotation))})\n"
            f"            double3 xformOp:translate = ({_fmt(position)})\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def hinge(name, body0, body1, low, high, *, stiffness, damping, target, rot0, rot1):
    """A hinge about both bodies' y axis at their origins (radians here; USD wants degrees)."""
    k, c = stiffness * math.pi / 180, damping * math.pi / 180
    return (f'        def PhysicsRevoluteJoint "{name}" (\n            prepend apiSchemas = ["PhysicsDriveAPI:angular"]\n        )\n        {{\n'
            f"            float drive:angular:physics:damping = {c:.6g}\n"
            f"            float drive:angular:physics:stiffness = {k:.6g}\n"
            f"            float drive:angular:physics:targetPosition = {math.degrees(target):.6g}\n"
            "            float drive:angular:physics:targetVelocity = 0\n"
            '            uniform token drive:angular:physics:type = "force"\n'
            '            uniform token physics:axis = "Y"\n'
            f"            rel physics:body0 = </Fixture/Actors/{body0}>\n"
            f"            rel physics:body1 = </Fixture/Actors/{body1}>\n"
            "            bool physics:collisionEnabled = 0\n"
            "            uniform bool physics:excludeFromArticulation = 1\n"
            "            point3f physics:localPos0 = (0, 0, 0)\n            point3f physics:localPos1 = (0, 0, 0)\n"
            f"            quatf physics:localRot0 = ({_fmt(rot0)})\n            quatf physics:localRot1 = ({_fmt(rot1)})\n"
            f"            float physics:lowerLimit = {math.degrees(low):.6g}\n"
            f"            float physics:upperLimit = {math.degrees(high):.6g}\n"
            "        }\n\n")


def material(name):
    color, roughness, metallic, key = MATERIALS[name]
    static, dynamic = m.FRICTION[key] if key else (.5, .5)
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {dynamic}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {static}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({_fmt(color)})\n"
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# -- geometry ---------------------------------------------------------------------------------------------------
def plate(i):
    """One plate: its box collider is the drawn sheet. The top plate carries a few drawn lines of 'text'."""
    text = box("sheet", "paper_a" if i % 2 == 0 else "paper_b", (0., 0., 0.), (m.SHEET_U, m.SHEET_V, m.PLATE_T), indent=12)
    if i == m.K - 1:
        for k in range(9):
            v = m.SHEET_V / 2 - .035 - .022 * k
            length = (.150, .165, .120)[k % 3]
            text += box(f"line_{k}", "ink", (-(.165 - length) / 2, v, m.PLATE_T / 2 + .00003), (length, .004, .00004),
                        collide=False, indent=12)
    return text


def clip_half(sign):
    """Colliders of one half of the clip in the clip frame (x = n across the paper, y = w along the width,
    z = u toward the lips). ``sign`` +1: the half on the +n side; -1: its mirror image."""
    p0 = np.array((m.BACK_HALF, 0.))
    p1 = np.array((m.LIP_R, m.JAW_L))
    jaw_len = float(np.linalg.norm(p1 - p0))
    jaw_turn = float(np.arctan2(p1[0] - p0[0], p1[1] - p0[1]))
    mid = (p0 + p1) / 2
    handle_len = m.HANDLE_L / np.cos(m.HANDLE_SPLAY)
    handle_mid = np.array((m.BACK_HALF + m.HANDLE_L * np.tan(m.HANDLE_SPLAY) / 2, -m.HANDLE_L / 2))
    text = box("jaw", "clip_black", (sign * mid[0], 0., mid[1]), (m.JAW_T, m.CLIP_W, jaw_len), turn_y=sign * jaw_turn)
    text += capsule("lip", "clip_black", (sign * m.LIP_R, 0., m.JAW_L), m.LIP_R, m.CLIP_W - 2 * m.LIP_R)
    text += box("back", "clip_black", (sign * (m.BACK_HALF + m.JAW_T / 2) / 2, 0., -m.JAW_T / 2),
                ((m.BACK_HALF + m.JAW_T / 2), m.CLIP_W, m.JAW_T))
    text += box("handle", "wire", (sign * handle_mid[0], 0., handle_mid[1]), (m.HANDLE_T + .0004, m.HANDLE_W, handle_len),
                turn_y=-sign * m.HANDLE_SPLAY)
    return text


def fixed():
    bx, by, bz = m.BLOCK
    fx, fy, fz = m.FIN
    middle = (m.BLOCK_NEAR_X + bx / 2, m.BLOCK_Y)
    if m.SUPPORT == "reams":             # one fixed box; the seam between the two reams and their label bands are drawn only
        text = box("block", "wrapper", (*middle, bz / 2), (bx, by, bz), indent=8)
        text += box("ream_seam", "ink", (*middle, bz / 2), (bx + .0006, by + .0006, .0012), collide=False, indent=8)
        for k, z in enumerate((bz / 4, 3 * bz / 4)):
            text += box(f"ream_band_{k}", "wrapper_band", (*middle, z), (bx + .0004, by + .0004, .018), collide=False, indent=8)
    else:
        text = box("block", "mdf", (*middle, bz / 2), (bx, by, bz), indent=8)
    text += box("fin", "acrylic", (m.FIN_XY[0], m.FIN_XY[1], fz / 2), (fx, fy, fz), indent=8)
    text += box("fin_foot", "acrylic", (m.FIN_XY[0], m.FIN_XY[1], .0015), (fx + .020, .040, .003), collide=False, indent=8)
    return text


def lift(pose):
    position, rotation = pose
    return np.asarray(position) + (0., 0., scene.TOP), rotation


def usd():
    bodies = ""
    for i, (name, pose) in enumerate(m.start_poses().items()):
        bodies += body(name, m.PLATE_MASS, *lift(pose), plate(i))
    start = m.clip_start()
    bodies += body("clip_a", m.CLIP_MASS / 2, *lift(start["clip_a"]), clip_half(1.), no_contact_with=("clip_b",))
    bodies += body("clip_b", m.CLIP_MASS / 2, *lift(start["clip_b"]), clip_half(-1.))
    joints = hinge("clip_hinge", "clip_b", "clip_a", 0., m.CLIP_OPEN, stiffness=m.CLIP_RATE, damping=m.CLIP_DAMPING,
                   target=-m.CLIP_PRELOAD / m.CLIP_RATE, rot0=(1, 0, 0, 0), rot1=(1, 0, 0, 0))
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n' + bodies + "    }\n\n"
            '    def Xform "Fixed"\n    {\n'
            f"        double3 xformOp:translate = (0, 0, {scene.TOP:.6g})\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate"]\n\n' + fixed() + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + joints + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD.name}: {m.K} plates of {1000 * m.PLATE_T:.2f} mm and {1000 * m.PLATE_MASS:.1f} g, "
          f"clip {m.clamp_force(m.STACK_T):.1f} N on the stack, {1000 * m.JAW_GAP_OPEN:.1f} mm open")
