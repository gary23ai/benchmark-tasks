"""Write cups.usda from cup_model.py:  python -m rdb_nested_cups.build_scene

Each cup is one rigid body with rigid colliders only. It collides as 24 flat wall panels of the true wall
thickness and a base disc, and is drawn as 24 slightly wider panels plus the same disc, so that the drawn wall
has no gaps. The collision panels are as wide as the wall at the base, where the cup is narrowest: wider boxes
would stick out past their neighbours. Two collision-only simplifications of the inner cup (cup_model.py says
why): its rim band, above the outer cup, is 24 blocks with the true outside surface, 12 mm deep inward, and
below the band its inside is solid (stacked cylinders just inside the wall).

The stuck fit is one breakable fixed joint between the two cups (cup_model.FIT_BREAK_FORCE_N); the cups'
walls do not touch while it holds.
"""

import math
from pathlib import Path

from . import cup_model as cup

USD = Path(__file__).with_name("cups.usda")
COLORS = {"outer": (0.0196, 0.1329, 0.2530), "inner": (0.9130, 0.5780, 0.0210)}  # linear RGB: blue, yellow
BAND_GAP = 0.0002  # the thin wall stops this far below the thick rim band
WALL_START = 0.0002  # the panels stop this far above the base plane, so the cup stands on its base disc
BASE_DISC_RADIUS = cup.BASE_RADIUS - cup.WALL  # inside the wall: the disc never touches the other cup


def _f(values):
    return "(" + ", ".join(f"{float(v) + 0.:.9g}" for v in values) + ")"


def _panel(name, material, index, half_width, *, collide, mass=None, bottom=WALL_START, top=cup.HEIGHT,
           thickness=cup.WALL):
    """One wall panel from height ``bottom`` to ``top``: a box leaning outward by the cup's half angle, turned
    ``index`` steps about the axis. Its outside face is the cup's surface; ``thickness`` goes inward."""
    a, phi = cup.HALF_ANGLE, 2. * math.pi * index / cup.SEGMENTS
    z_mid = (bottom + top) / 2.
    half_length = (top - bottom) / (2. * math.cos(a))
    radius = cup.outside_radius(z_mid) - thickness / (2. * math.cos(a))  # middle of the box
    # Rotation about z by phi after rotation about y by a, as a WXYZ quaternion.
    quat = (math.cos(phi / 2) * math.cos(a / 2), -math.sin(phi / 2) * math.sin(a / 2),
            math.cos(phi / 2) * math.sin(a / 2), math.sin(phi / 2) * math.cos(a / 2))
    schemas = ["MaterialBindingAPI"] + (["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMassAPI"] if collide else [])
    body = [f"rel material:binding = </Fixture/Materials/{material}>"]
    if collide:
        body += [f"rel material:binding:physics = </Fixture/Materials/{material}>",
                 f"float physics:mass = {mass:.9g}",
                 "float physxCollision:contactOffset = 0.0015", "float physxCollision:restOffset = 0",
                 'token visibility = "invisible"']
    body += ["double size = 2", f"quatf xformOp:orient = {_f(quat)}",
             f"float3 xformOp:scale = {_f((thickness / 2., half_width, half_length))}",
             f"double3 xformOp:translate = {_f((radius * math.cos(phi), radius * math.sin(phi), z_mid))}",
             'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]']
    return _prim("Cube", name, schemas, body)


def _prim(kind, name, schemas, body):
    pad = " " * 12
    head = [f'{pad}def {kind} "{name}" (', f"{pad}    prepend apiSchemas = {schemas!r}".replace("'", '"'), f"{pad})", f"{pad}{{"]
    return "\n".join(head + [f"{pad}    {line}" for line in body] + [f"{pad}}}", ""]) + "\n"


def _cup(name, start):
    wall_area = math.pi * (cup.RIM_RADIUS + cup.BASE_RADIUS) * math.hypot(cup.HEIGHT, cup.RIM_RADIUS - cup.BASE_RADIUS)
    base_mass = cup.MASS * math.pi * cup.BASE_RADIUS ** 2 / (wall_area + math.pi * cup.BASE_RADIUS ** 2)
    panel_mass = (cup.MASS - base_mass) / cup.SEGMENTS
    narrow = cup.outside_radius(WALL_START) * math.tan(math.pi / cup.SEGMENTS)
    wide = cup.RIM_RADIUS * math.tan(math.pi / cup.SEGMENTS)
    world = (start[0], start[1], start[2] + cup.TABLE_TOP_WORLD_Z)
    text = f'''        def Xform "{name}" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "PhysicsMassAPI"]
        )
        {{
            float physics:mass = {cup.MASS:.9g}
            bool physics:rigidBodyEnabled = 1
            float physxContactReport:threshold = 0
            float physxRigidBody:maxDepenetrationVelocity = 0.75
            int physxRigidBody:solverPositionIterationCount = 32
            int physxRigidBody:solverVelocityIterationCount = 8
            quatf xformOp:orient = (1, 0, 0, 0)
            double3 xformOp:translate = {_f(world)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

'''
    banded = name == "inner"
    if banded:
        base_mass -= cup.PLUG_LAYERS * .0001  # the solid inside carries a token mass; the cup's total is unchanged
    wall_top = cup.BAND_START - BAND_GAP if banded else cup.HEIGHT
    band_share = (cup.HEIGHT - cup.BAND_START) / cup.HEIGHT if banded else 0.
    band_width = cup.outside_radius(cup.BAND_START) * math.tan(math.pi / cup.SEGMENTS)
    for i in range(cup.SEGMENTS):
        text += _panel(f"{name}_wall{i}", f"{name}_wall", i, narrow, collide=True,
                       mass=panel_mass * (1. - band_share), top=wall_top)
        if banded:
            text += _panel(f"{name}_band{i}", f"{name}_base", i, band_width, collide=True,
                           mass=panel_mass * band_share, bottom=cup.BAND_START, thickness=cup.BAND_THICKNESS)
        text += _panel(f"{name}_skin{i}", f"{name}_wall", i, wide, collide=False)
    text += _prim("Cylinder", f"{name}_base",
                  ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMassAPI"],
                  ['uniform token axis = "Z"', f"double height = {cup.WALL:.9g}", f"double radius = {BASE_DISC_RADIUS:.9g}",
                   f"rel material:binding = </Fixture/Materials/{name}_base>",
                   f"rel material:binding:physics = </Fixture/Materials/{name}_base>",
                   f"float physics:mass = {base_mass:.9g}",
                   "float physxCollision:contactOffset = 0.0015", "float physxCollision:restOffset = 0",
                   "quatf xformOp:orient = (1, 0, 0, 0)", f"double3 xformOp:translate = {_f((0, 0, cup.WALL / 2.))}",
                   'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]'])
    if banded:  # the solid inside of the inner cup below the rim band
        edges = [cup.WALL + (cup.PLUG_TOP - cup.WALL) * k / cup.PLUG_LAYERS for k in range(cup.PLUG_LAYERS + 1)]
        for k, (low, high) in enumerate(zip(edges[:-1], edges[1:])):
            radius = cup.outside_radius(low) - cup.WALL / math.cos(cup.HALF_ANGLE) - cup.PLUG_CLEARANCE
            text += _prim("Cylinder", f"{name}_plug{k}",
                          ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMassAPI"],
                          ['uniform token axis = "Z"', f"double height = {high - low:.9g}", f"double radius = {radius:.9g}",
                           f"rel material:binding = </Fixture/Materials/{name}_base>",
                           f"rel material:binding:physics = </Fixture/Materials/{name}_base>",
                           "float physics:mass = 0.0001",
                           "float physxCollision:contactOffset = 0.0015", "float physxCollision:restOffset = 0",
                           'token visibility = "invisible"',
                           "quatf xformOp:orient = (1, 0, 0, 0)",
                           f"double3 xformOp:translate = {_f((0, 0, (low + high) / 2.))}",
                           'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]'])
    return text + "        }\n\n"


def _fit_joint():
    """The stuck fit: a fixed joint between the cups that breaks for good above a set force or moment."""
    in_inner = cup.FIT_FRAME_HEIGHT - cup.SEATED_PITCH
    return f'''        def PhysicsFixedJoint "fit"
        {{
            rel physics:body0 = </Fixture/Actors/outer>
            rel physics:body1 = </Fixture/Actors/inner>
            float physics:breakForce = {cup.FIT_BREAK_FORCE_N:.9g}
            float physics:breakTorque = {cup.FIT_BREAK_TORQUE_NM:.9g}
            bool physics:collisionEnabled = 1
            uniform bool physics:excludeFromArticulation = 1
            bool physics:jointEnabled = 1
            point3f physics:localPos0 = {_f((0, 0, cup.FIT_FRAME_HEIGHT))}
            point3f physics:localPos1 = {_f((0, 0, in_inner))}
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
        }}

'''


def _probe_body(name, half, radius, height, mass, base):
    """A plain rigid cylinder or box standing on the table, for the diagnostic probe only."""
    world = (base[0], base[1], base[2] + cup.TABLE_TOP_WORLD_Z)
    head = f'''        def Xform "{name}" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "PhysicsMassAPI"]
        )
        {{
            float physics:mass = {mass:.9g}
            bool physics:rigidBodyEnabled = 1
            float physxContactReport:threshold = 0
            float physxRigidBody:maxDepenetrationVelocity = 0.75
            int physxRigidBody:solverPositionIterationCount = 32
            int physxRigidBody:solverVelocityIterationCount = 8
            quatf xformOp:orient = (1, 0, 0, 0)
            double3 xformOp:translate = {_f(world)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

'''
    common = ["rel material:binding = </Fixture/Materials/probe>",
              "rel material:binding:physics = </Fixture/Materials/probe>",
              "float physxCollision:contactOffset = 0.0015", "float physxCollision:restOffset = 0",
              "quatf xformOp:orient = (1, 0, 0, 0)", f"double3 xformOp:translate = {_f((0, 0, height / 2.))}"]
    schemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]
    if half is None:
        shape = _prim("Cylinder", f"{name}_solid", schemas,
                      ['uniform token axis = "Z"', f"double height = {height:.9g}", f"double radius = {radius:.9g}"]
                      + common + ['uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]'])
    else:
        shape = _prim("Cube", f"{name}_solid", schemas,
                      ["double size = 2"] + common + [f"float3 xformOp:scale = {_f(half)}",
                       'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]'])
    return head + shape + "        }\n\n"


def _material(name, color):
    return f'''        def Material "{name}" (
            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        )
        {{
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
            float physics:dynamicFriction = {cup.FRICTION:.9g}
            float physics:restitution = 0
            float physics:staticFriction = {cup.FRICTION:.9g}
            uniform token physxMaterial:frictionCombineMode = "min"

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {_f(color)}
                float inputs:roughness = 0.54
                token outputs:surface
            }}
        }}

'''


def text():
    out = ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
           'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n')
    for name, start in zip(cup.CUPS, cup.START, strict=True):
        out += _cup(name, start)
    if cup.PROBE:
        for name, spec in cup.PROBE_BODIES.items():
            out += _probe_body(name, *spec)
    out += '    }\n\n    def Scope "Static"\n    {\n    }\n\n    def Scope "Joints"\n    {\n'
    out += _fit_joint()
    out += '    }\n\n    def Scope "Materials"\n    {\n'
    for name in cup.CUPS:
        out += _material(f"{name}_wall", COLORS[name])
        out += _material(f"{name}_base", COLORS[name])
    if cup.PROBE:
        out += _material("probe", (0.25, 0.25, 0.25))
    return out + "    }\n}\n"


if __name__ == "__main__":
    USD.write_text(text())
    print(f"wrote {USD.name}: inner cup {1000 * cup.SEATED_PITCH:.2f} mm above the outer cup's base, "
          f"bond breaks at {cup.FIT_BREAK_FORCE_N:.1f} N or {cup.FIT_BREAK_TORQUE_NM:.2f} N m")
