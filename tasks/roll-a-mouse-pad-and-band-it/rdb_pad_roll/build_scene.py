"""Regenerate ``pad.usda`` from ``pad_model.py``.   Run:  python -m rdb_pad_roll.build_scene

In the file:

* ``/Fixture/Static``: the carton and the band's cradle, colliders fixed to the world.
* ``/Fixture/Actors/strip_0 .. strip_24`` and ``link_1 .. link_24``: the mouse pad, one floating articulation.
  Neighbouring strips are joined through a link: a sprung hinge along the strip (the pad's bending) and then a
  sprung slide along the same line (see ``pad_model.SHEAR_K``). Each strip collides as two boxes, a rubber
  underside and a cloth top, and is drawn as a full-width slab.
* ``/Fixture/Actors/band``: the paper band, one free rigid body whose wall is 16 flat boxes round an open bore.
"""

import math

from . import fixed_scene as scene
from . import pad_model as m

TOP = scene.TOP
# colour (linear), roughness, friction
MATERIALS = {
    "rubber": ((.012, .012, .014), .9, m.FRICTION_RUBBER),
    "cloth": ((.03, .12, .33), .95, m.FRICTION_CLOTH),
    "stitch": ((.55, .62, .75), .9, m.FRICTION_CLOTH),
    "carton": ((.80, .79, .74), .8, m.FRICTION_CARTON),
    "carton_tape": ((.55, .43, .27), .5, m.FRICTION_CARTON),
    "kraft": ((.36, .23, .11), .85, m.FRICTION_BAND),
    "kraft_end": ((.50, .36, .20), .9, m.FRICTION_BAND),
    "cradle": ((.10, .10, .11), .6, m.FRICTION_CRADLE),
}
PREFIX = "</Fixture/Materials/"
CLOTH_T = .0006            # drawn thickness of the cloth face


def cube(name, material, centre, half, *, collide=True, visible=True, roll=0., pad="            "):
    """A box; ``roll`` turns it about its own x axis (radians)."""
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = f'{pad}def Cube "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    text += f"{pad}    rel material:binding = {PREFIX}{material}>\n"
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = 0.001\n{pad}    float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{pad}    uniform token purpose = "guide"\n'
    text += (f"{pad}    double size = 2\n"
             f"{pad}    quatf xformOp:orient = ({math.cos(roll / 2):.8g}, {math.sin(roll / 2):.8g}, 0, 0)\n"
             f"{pad}    float3 xformOp:scale = ({half[0]:.6g}, {half[1]:.6g}, {half[2]:.6g})\n"
             f"{pad}    double3 xformOp:translate = ({centre[0]:.6g}, {centre[1]:.6g}, {centre[2]:.6g})\n"
             f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n{pad}}}\n\n')
    return text


def slab(name, material, x, y, z, **kwargs):
    """A box given by its (low, high) extents on each axis."""
    return cube(name, material, [(a + b) / 2 for a, b in (x, y, z)], [(b - a) / 2 for a, b in (x, y, z)], **kwargs)


def body(name, mass, position, children, *, root=False, inertia=None, damping=(.1, .5), filtered=(), pad="        "):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    extra = ""
    if root:
        apis += ['"PhysicsArticulationRootAPI"', '"PhysxArticulationAPI"']
        extra = (f"{pad}    bool physxArticulation:enabledSelfCollisions = 1\n"
                 f"{pad}    int physxArticulation:solverPositionIterationCount = {m.SOLVER_ITERATIONS[0]}\n"
                 f"{pad}    int physxArticulation:solverVelocityIterationCount = {m.SOLVER_ITERATIONS[1]}\n"
                 f"{pad}    float physxArticulation:sleepThreshold = 0\n"
                 f"{pad}    float physxArticulation:stabilizationThreshold = 0\n")
    if filtered:
        apis.append('"PhysicsFilteredPairsAPI"')
        extra += f"{pad}    rel physics:filteredPairs = [{', '.join(f'<{p}>' for p in filtered)}]\n"
    if inertia is not None:
        extra += f"{pad}    float3 physics:diagonalInertia = ({inertia[0]:.6g}, {inertia[1]:.6g}, {inertia[2]:.6g})\n"
    return (f'{pad}def Xform "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
            f"{pad}    float physics:mass = {mass:.6g}\n{pad}    bool physics:rigidBodyEnabled = 1\n" + extra +
            f"{pad}    point3f physics:centerOfMass = (0, 0, 0)\n"
            f"{pad}    float physxContactReport:threshold = 0\n"
            f"{pad}    float physxRigidBody:maxDepenetrationVelocity = 1\n"
            f"{pad}    float physxRigidBody:sleepThreshold = 0\n"
            f"{pad}    float physxRigidBody:angularDamping = {damping[1]}\n"
            f"{pad}    float physxRigidBody:linearDamping = {damping[0]}\n"
            f"{pad}    int physxRigidBody:solverPositionIterationCount = {m.SOLVER_ITERATIONS[0]}\n"
            f"{pad}    int physxRigidBody:solverVelocityIterationCount = {m.SOLVER_ITERATIONS[1]}\n"
            f"{pad}    quatf xformOp:orient = (1, 0, 0, 0)\n"
            f"{pad}    double3 xformOp:translate = ({position[0]:.6g}, {position[1]:.6g}, {position[2]:.6g})\n"
            f'{pad}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}{pad}}}\n\n")


def joint(kind, name, body0, body1, anchor0, anchor1, axis, limits, *, stiffness=0., damping=0., armature=0.,
          pad="        "):
    """``kind`` is "Revolute" (limits and stiffness per degree) or "Prismatic" (metres)."""
    drive = "angular" if kind == "Revolute" else "linear"
    return (f'{pad}def Physics{kind}Joint "{name}" (\n'
            f'{pad}    prepend apiSchemas = ["PhysicsDriveAPI:{drive}", "PhysxJointAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    float drive:{drive}:physics:damping = {damping:.7g}\n"
            f"{pad}    float drive:{drive}:physics:stiffness = {stiffness:.7g}\n"
            f"{pad}    float drive:{drive}:physics:targetPosition = 0\n"
            f"{pad}    float drive:{drive}:physics:targetVelocity = 0\n"
            f'{pad}    uniform token drive:{drive}:physics:type = "force"\n'
            f'{pad}    uniform token physics:axis = "{axis}"\n'
            f"{pad}    rel physics:body0 = <{body0}>\n{pad}    rel physics:body1 = <{body1}>\n"
            f"{pad}    point3f physics:localPos0 = ({anchor0[0]:.6g}, {anchor0[1]:.6g}, {anchor0[2]:.6g})\n"
            f"{pad}    point3f physics:localPos1 = ({anchor1[0]:.6g}, {anchor1[1]:.6g}, {anchor1[2]:.6g})\n"
            f"{pad}    quatf physics:localRot0 = (1, 0, 0, 0)\n{pad}    quatf physics:localRot1 = (1, 0, 0, 0)\n"
            f"{pad}    float physics:lowerLimit = {limits[0]:.7g}\n{pad}    float physics:upperLimit = {limits[1]:.7g}\n"
            f"{pad}    float physxJoint:armature = {armature:.7g}\n"
            f"{pad}    float physxJoint:maxJointVelocity = {1440 if kind == 'Revolute' else 1}\n{pad}}}\n\n")


def material(name):
    color, roughness, friction = MATERIALS[name]
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n"
            f"                float inputs:metallic = 0\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


# ---------------------------------------------------------------------------------------------------------
def static_parts():
    pad = "        "
    x0, y0 = m.CARTON_X0, m.CARTON_Y0
    x1, y1 = x0 + m.CARTON_L, y0 + m.CARTON_W
    text = slab("carton", "carton", (x0, x1), (y0, y1), (TOP, TOP + m.CARTON_H), pad=pad)
    # the tape along the top's centre seam and down both ends: drawn only
    yc = (y0 + y1) / 2
    text += slab("tape_top", "carton_tape", (x0, x1), (yc - .024, yc + .024), (TOP + m.CARTON_H, TOP + m.CARTON_H + .0002),
                 collide=False, pad=pad)
    for name, x in (("tape_near", x0 - .0002), ("tape_far", x1)):
        text += slab(name, "carton_tape", (x, x + .0002), (yc - .024, yc + .024), (TOP + m.CARTON_H - .05, TOP + m.CARTON_H),
                     collide=False, pad=pad)
    # cradle: two V saddles on a base strip
    cx, cy, cz = m.CRADLE_X, m.CRADLE_Y, TOP + m.BAND_REST_Z
    for k, dx in enumerate(m.SADDLE_X):
        for side, (y, z, roll) in enumerate(m.saddle_faces()):
            text += cube(f"saddle_{k}_{side}", "cradle", (cx + dx, cy + y, cz + z),
                         (m.SADDLE_T / 2, m.SADDLE_FACE / 2, m.SADDLE_PLATE / 2), roll=roll, pad=pad)
        text += slab(f"saddle_{k}_foot", "cradle", (cx + dx - m.SADDLE_T / 2, cx + dx + m.SADDLE_T / 2),
                     (cy - .034, cy + .034), (TOP, TOP + .008), pad=pad)
    text += slab("cradle_base", "cradle", (cx + m.SADDLE_X[0] - m.SADDLE_T / 2, cx + m.SADDLE_X[1] + m.SADDLE_T / 2),
                 (cy - .012, cy + .012), (TOP, TOP + .006), collide=False, pad=pad)
    return text


def pad_bodies():
    text = ""
    hw, hy, ht = m.STRIP_W / 2, m.PAD_W / 2, m.PAD_T / 2
    here = "/Fixture/Actors/"
    for i in range(m.STRIPS):
        strip = cube("rubber", "rubber", (0., 0., -ht / 2), (m.COLLIDER_W / 2, hy, ht / 2), visible=False)
        strip += cube("cloth", "cloth", (0., 0., ht / 2), (m.COLLIDER_W / 2, hy, ht / 2), visible=False)
        strip += slab("foam", "rubber", (-hw, hw), (-hy, hy), (-ht, ht - CLOTH_T), collide=False)
        strip += slab("face", "cloth", (-hw, hw), (-hy, hy), (ht - CLOTH_T, ht), collide=False)
        # the stitched border: along both long edges, and across the two end strips
        for side, y in (("r", -hy + .004), ("l", hy - .004)):
            strip += slab(f"stitch_{side}", "stitch", (-hw, hw), (y - .001, y + .001), (ht, ht + .00008), collide=False)
        if i in (0, m.STRIPS - 1):
            x = (-hw + .004) if i == 0 else (hw - .004)
            strip += slab("stitch_end", "stitch", (x - .001, x + .001), (-hy + .004, hy - .004), (ht, ht + .00008),
                          collide=False)
        near = [f"{here}strip_{j}" for j in (i - 1, i + 1) if 0 <= j < m.STRIPS]
        text += body(f"strip_{i}", m.STRIP_MASS, m.PAD_START[i] + (0., 0., TOP), strip, root=(i == m.ROOT),
                     inertia=m.STRIP_INERTIA, filtered=near)
    for i in range(1, m.STRIPS):                 # link_i rides on the hinge line between strip_{i-1} and strip_i
        x = m.PAD_X0 + i * m.STRIP_W
        text += body(f"link_{i}", m.LINK_MASS, (x, m.PAD_Y, m.PAD_Z + TOP), "", inertia=(2e-7, 2e-7, 2e-7))
    return text


def band_body():
    half_l = m.BAND_L / 2
    parts = ""
    for k, (angle, radius, half_w) in enumerate(m.band_staves()):
        # a stave lies along the band (x), its flat face toward the axis; turn it about x to its place
        centre = (0., -radius * math.sin(angle), radius * math.cos(angle))
        parts += cube(f"stave_{k:02d}", "kraft", centre, (half_l, half_w, m.BAND_WALL / 2), roll=angle, visible=False)
    parts += band_mesh()
    return body("band", m.BAND_MASS, m.BAND_START + (0., 0., TOP), parts, damping=(.05, .3))


def band_mesh(n=48, pad="            "):
    """The drawn band: outer and inner skins and the two cut ends."""
    ri, ro, h = m.BAND_ID / 2, m.BAND_OD / 2, m.BAND_L / 2
    points, counts, indices = [], [], []
    for x in (-h, h):
        for r in (ro, ri):
            points += [(x, r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    # rings: 0 outer at -h, 1 inner at -h, 2 outer at +h, 3 inner at +h
    def quad(a, b, k, flip=False):
        j = (k + 1) % n
        q = [a * n + k, a * n + j, b * n + j, b * n + k]
        return q[::-1] if flip else q
    for k in range(n):
        for a, b, flip in ((0, 2, True), (1, 3, False), (0, 1, False), (2, 3, True)):
            indices += quad(a, b, k, flip)
            counts.append(4)
    pts = ", ".join(f"({p[0]:.5g}, {p[1]:.5g}, {p[2]:.5g})" for p in points)
    return (f'{pad}def Mesh "skin" (\n{pad}    prepend apiSchemas = ["MaterialBindingAPI"]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}kraft>\n"
            f"{pad}    uniform bool doubleSided = 1\n"
            f"{pad}    int[] faceVertexCounts = [{', '.join(map(str, counts))}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(map(str, indices))}]\n"
            f"{pad}    point3f[] points = [{pts}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def pad_joints():
    per_deg = math.pi / 180.
    here = "/Fixture/Actors/"
    hw = m.STRIP_W / 2
    text = ""
    for i in range(1, m.STRIPS):
        # the tree grows outward from the root strip, so each joint's body0 is the side nearer the root
        if i <= m.ROOT:
            inner, outer, a_in, a_out = f"strip_{i}", f"strip_{i - 1}", (-hw, 0., 0.), (hw, 0., 0.)
        else:
            inner, outer, a_in, a_out = f"strip_{i - 1}", f"strip_{i}", (hw, 0., 0.), (-hw, 0., 0.)
        text += joint("Revolute", f"fold_{i}", here + inner, here + f"link_{i}", a_in, (0., 0., 0.), "Y",
                      (-m.HINGE_LIMIT_DEG, m.HINGE_LIMIT_DEG), stiffness=m.HINGE_K * per_deg,
                      damping=m.HINGE_DAMPING * per_deg, armature=m.HINGE_ARMATURE)
        text += joint("Prismatic", f"slide_{i}", here + f"link_{i}", here + outer, (0., 0., 0.), a_out, "Y",
                      (-m.SHEAR_LIMIT, m.SHEAR_LIMIT), stiffness=m.SHEAR_K, damping=m.SHEAR_DAMPING, armature=1e-4)
    return text


def usd():
    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n'
            '    def Scope "Static"\n    {\n' + static_parts() + "    }\n\n"
            '    def Scope "Actors"\n    {\n' + pad_bodies() + band_body() + "    }\n\n"
            '    def Scope "Joints"\n    {\n' + pad_joints() + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print(f"wrote {scene.USD} ({len(usd().splitlines())} lines)")
