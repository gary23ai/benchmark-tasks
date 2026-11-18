"""Write window.usda from cleaner_model.py:  python -m rdb_window_wall.build_scene

The wall (sill, head, pier, stile and two feet) and the glass of the fixed light are fixed box colliders;
the open light is a real hole between them. Each half of the cleaner is one rigid body: a triangular pad
and shell (convex prisms) and a handle bar on two posts (boxes). The marker band is drawn as 180 squares of
5 mm with no collider; the adapter hides a square once the scorer counts it as cleaned.

What is rendered is partly drawn (drawn.py, no physics): the halves' colliders are hidden and drawn instead
as the product looks (rounded body, separate pad, D handle over the gripped bar), and the wall gets drawn
depth, a window frame, beads, architrave, window board and skirting. The colliders are unchanged.
"""

from pathlib import Path

import numpy as np

from . import cleaner_model as m
from . import drawn

USD = Path(__file__).with_name("window.usda")
TOP = m.TABLE_TOP_WORLD_Z
COLORS = {  # linear RGB
    "inside_shell": (0.03, 0.20, 0.60), "outside_shell": (0.80, 0.80, 0.78), "pad": (0.75, 0.62, 0.20),
    "glass": (0.62, 0.80, 0.82), "wall": (0.85, 0.85, 0.82), "marker": (0.01, 0.01, 0.01),
}
BODY = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "PhysicsMassAPI"]
SOLID = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMassAPI"]
FIXED = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"]


def _f(values):
    return "(" + ", ".join(f"{float(v) + 0.:.9g}" for v in values) + ")"


def _prim(kind, name, schemas, body, indent=12):
    pad = " " * indent
    head = [f'{pad}def {kind} "{name}" (', f"{pad}    prepend apiSchemas = {schemas!r}".replace("'", '"'), f"{pad})", f"{pad}{{"]
    return "\n".join(head + [f"{pad}    {line}" for line in body] + [f"{pad}}}", ""]) + "\n"


def _bind(material, collide, hidden=False):
    lines = [f"rel material:binding = </Fixture/Materials/{material}>"]
    if hidden:  # a collider that is not rendered: the drawn shape (drawn.py) shows it instead
        lines.append('token visibility = "invisible"')
    if collide:
        lines += [f"rel material:binding:physics = </Fixture/Materials/{material}>",
                  "float physxCollision:contactOffset = 0.002", "float physxCollision:restOffset = 0"]
    return lines


def _box(name, material, size, centre, *, mass=None, collide=True, indent=12, quat_xyzw=(0., 0., 0., 1.),
         hidden=False):
    schemas = (SOLID if mass is not None else FIXED) if collide else ["MaterialBindingAPI"]
    body = _bind(material, collide, hidden) + ([f"float physics:mass = {mass:.9g}"] if mass is not None else [])
    x, y, z, w = quat_xyzw
    body += ["double size = 2", f"quatf xformOp:orient = {_f((w, x, y, z))}",
             f"float3 xformOp:scale = {_f(np.asarray(size) / 2)}",
             f"double3 xformOp:translate = {_f(centre)}",
             'uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]']
    return _prim("Cube", name, schemas, body, indent)


def _fixed(name, material, size, centre_pane, **kwargs):
    """A box given in pane coordinates (u, n, z), written in the tabletop frame."""
    centre = m.to_world(centre_pane) + (0., 0., TOP)
    return _box(name, material, size, centre, indent=8, quat_xyzw=m.PANE_QUAT, **kwargs)


def wall_boxes():
    """Every fixed box of the wall and window as ((u0, u1), (n0, n1), (z0, z1)) in pane coordinates."""
    n = m.WALL_N
    glass = (m.FACE_Y["outside"], m.FACE_Y["inside"])
    foot_n = (m.WALL_MID_N - m.FOOT_SIZE[1] / 2, m.WALL_MID_N + m.FOOT_SIZE[1] / 2)
    boxes = {
        "sill": (m.WALL_U, n, (0., m.OPENING_Z[0])),
        "head": (m.WALL_U, n, (m.OPENING_Z[1], m.WALL_H)),
        "pier_right": ((m.WALL_U[0], m.OPENING_U[0]), n, m.OPENING_Z),
        "stile": (m.STILE_U, n, m.OPENING_Z),
        "pier_left": ((m.PANE_X[1], m.WALL_U[1]), n, m.OPENING_Z),
        "pane": (m.PANE_X, glass, m.PANE_Z),
    }
    for tag, u in zip(("a", "b"), m.FOOT_U):
        # The feet lie under the wall's two ends, outside the piers' footprint only below the sill's height.
        boxes[f"foot_{tag}_in"] = ((u - m.FOOT_SIZE[0] / 2, u + m.FOOT_SIZE[0] / 2), (n[1], foot_n[1]), (0., m.FOOT_SIZE[2]))
        boxes[f"foot_{tag}_out"] = ((u - m.FOOT_SIZE[0] / 2, u + m.FOOT_SIZE[0] / 2), (foot_n[0], n[0]), (0., m.FOOT_SIZE[2]))
    return boxes


def prism_points(z0, z1):
    return [(x, y, z) for z in (z0, z1) for x, y in m.TRIANGLE]


def _prism(name, material, z0, z1, mass):
    points = prism_points(z0, z1)
    lo, hi = np.min(points, axis=0), np.max(points, axis=0)
    body = _bind(material, True, hidden=True) + [
        f"float physics:mass = {mass:.9g}", 'uniform token physics:approximation = "convexHull"',
        'uniform token subdivisionScheme = "none"',
        f"float3[] extent = [{_f(lo)}, {_f(hi)}]",
        "int[] faceVertexCounts = [3, 3, 4, 4, 4]",
        # bottom (seen from below), top, then the three sides; all wound counter-clockwise seen from outside
        "int[] faceVertexIndices = [0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5]",
        "point3f[] points = [" + ", ".join(_f(p) for p in points) + "]"]
    return _prim("Mesh", name, SOLID + ["PhysicsMeshCollisionAPI"], body)


def _half(name):
    world = (m.START[name][0], m.START[name][1], m.START[name][2] + TOP)
    x, y, z, w = m.START_QUAT[name]
    text = f'''        def Xform "{name}" (
            prepend apiSchemas = {BODY!r}
        )
        {{
            float physics:mass = {m.MASS:.9g}
            bool physics:rigidBodyEnabled = 1
            float physxContactReport:threshold = 0
            bool physxRigidBody:enableCCD = 1
            float physxRigidBody:maxDepenetrationVelocity = 0.75
            int physxRigidBody:solverPositionIterationCount = 32
            int physxRigidBody:solverVelocityIterationCount = 8
            quatf xformOp:orient = {_f((w, x, y, z))}
            double3 xformOp:translate = {_f(world)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

'''.replace("'", '"')
    shell = f"{name}_shell"
    text += _prism("pad", "pad", 0., m.PAD_T, m.PART_MASS["pad"])
    text += _prism("shell", shell, m.PAD_T, m.BODY_T, m.PART_MASS["shell"])
    text += _box("bar", "handle_" + name, m.BAR_SIZE, m.GRIP_POINT, mass=m.PART_MASS["bar"], hidden=True)
    for sign, tag in ((-1., "a"), (1., "b")):
        text += _box(f"post_{tag}", "handle_" + name, m.POST_SIZE,
                     (sign * m.POST_X, 0., (m.BODY_T + m.BAR_Z[0]) / 2), mass=m.PART_MASS["post"], hidden=True)
    # What is rendered instead (no collider, no mass): drawn.py.
    text += drawn.mesh_text("Mesh", "drawn_pad", "pad_cloth", drawn.pad_mesh(), 12)
    text += drawn.mesh_text("Mesh", "drawn_body", shell, drawn.body_mesh(), 12)
    text += drawn.mesh_text("Mesh", "drawn_handle", "handle_" + name, drawn.handle_mesh(), 12)
    return text + "        }\n\n"


def _material(name, color, friction, *, combine="average", opacity=1., roughness=.5):
    return f'''        def Material "{name}" (
            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        )
        {{
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
            float physics:dynamicFriction = {friction:.9g}
            float physics:restitution = 0
            float physics:staticFriction = {friction:.9g}
            uniform token physxMaterial:frictionCombineMode = "{combine}"

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {_f(color)}
                float inputs:opacity = {opacity:.9g}
                float inputs:roughness = {roughness:.9g}
                token outputs:surface
            }}
        }}

'''


def _look(name, color, *, roughness=.5):
    """A material for drawn details only: no physics material."""
    return f'''        def Material "{name}"
        {{
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>

            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {_f(color)}
                float inputs:roughness = {roughness:.9g}
                token outputs:surface
            }}
        }}

'''


def text():
    out = ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
           'def Xform "Fixture"\n{\n    def Scope "Actors"\n    {\n')
    for name in m.HALVES:
        out += _half(name)
    out += '    }\n\n    def Scope "Static"\n    {\n'
    for name, (u, n, z) in wall_boxes().items():
        out += _fixed(name, "glass" if name == "pane" else "wall", (u[1] - u[0], n[1] - n[0], z[1] - z[0]),
                      (sum(u) / 2, sum(n) / 2, sum(z) / 2))
    out += '    }\n\n    def Scope "Drawn"\n    {\n'
    for name, (u, n, z, look) in drawn.wall_shapes().items():
        out += _fixed(f"drawn_{name}", look, (u[1] - u[0], n[1] - n[0], z[1] - z[0]),
                      (sum(u) / 2, sum(n) / 2, sum(z) / 2), collide=False)
    out += '    }\n\n    def Scope "Marks"\n    {\n'
    for row, z in enumerate(m.CELL_Z):
        for col, u in enumerate(m.CELL_U):
            out += _fixed(f"cell_{row}_{col:02d}", "marker", (m.CELL, 0.0004, m.CELL),
                          (u, m.FACE_Y["outside"] - 0.0003, z), collide=False)
    out += '    }\n\n    def Scope "Materials"\n    {\n'
    out += _material("pad", COLORS["pad"], m.PAD_FRICTION, combine="min", roughness=.9)
    for name in m.HALVES:
        out += _material(f"{name}_shell", COLORS[f"{name}_shell"], m.SHELL_FRICTION)
        out += _material(f"handle_{name}", COLORS[f"{name}_shell"], m.HANDLE_FRICTION)
    out += _material("glass", COLORS["glass"], m.GLASS_FRICTION, opacity=.25, roughness=.05)
    out += _material("wall", COLORS["wall"], m.WALL_FRICTION, roughness=.7)
    out += _material("marker", COLORS["marker"], .5)
    for name, color in {**drawn.WALL_COLORS, **drawn.HALF_COLORS}.items():  # drawn details: no physics
        out += _look(name, color, roughness=.95 if name == "pad_cloth" else .6)
    return out + "    }\n}\n"


if __name__ == "__main__":
    USD.write_text(text())
    print(f"wrote {USD.name}: {len(wall_boxes())} fixed boxes, {m.CELLS} band squares, "
          f"pull through {1000 * m.PANE_T:.0f} mm glass {m.hold_force()[0]:.1f} N, "
          f"pole strength {m.POLE_STRENGTH:.1f} A m")
