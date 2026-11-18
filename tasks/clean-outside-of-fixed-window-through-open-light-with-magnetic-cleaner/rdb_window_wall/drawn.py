"""Drawn details: what the scene looks like, with no effect on the physics. NumPy only.

Nothing here has a collider, a mass or a physics material. It is only rendered.

- The cleaner halves. Each is drawn as the product looks: a triangle with rounded corners and a rounded
  (bevelled) top edge, a separate cleaning pad a little smaller than the body so the body's rim shows round
  it, and a D-shaped handle: a grip section with rounded legs that flare into the body. The colliders that
  were run (the sharp triangular pad and shell, and the handle bar on two posts) are kept exactly and are
  hidden from the render where the drawn shape covers them. The drawn grip section has the bar's own
  24 x 10 mm section in the same place, so the gripper is seen closing on what it really holds; the drawn
  legs enclose the posts.
- The wall. The plywood board and the glass are the colliders that were run. Drawn on them: the wall's depth
  outdoors (so the wall reads as a wall about 80 mm thick, with the window set in a reveal), a white window
  frame lining the cut-out, a mullion over the stile, glazing beads round the glass, an architrave round the
  window and an interior window board indoors, and a skirting board. None of it narrows the open light by
  more than 1 mm, and none of it stands in the space the demonstration's arms or halves pass through
  (tests/test_scene.py and the README give the clearances).

Every shape is in pane coordinates (u, n, z) for the wall, or in a half's own frame for the halves.
"""

import numpy as np

from . import cleaner_model as m

# -- the halves --------------------------------------------------------------------------------
CORNER_R = 0.014  # radius of the body's rounded corners, in plan
BEVEL = 0.006  # radius of the rounded edge round the top of the body
RIM_LIFT = 0.0008  # the body's lower edge stands this far above the pad face
PAD_INSET = 0.0025  # the pad is this much smaller than the body all round, so the rim shows as a border
ARC_STEPS = 8
BEVEL_STEPS = 4
# The D handle: grip section over the bar, legs over the posts, a fillet between them, and a flared foot.
HANDLE_LEG_X = m.POST_X  # the legs' centre line, over the posts
HANDLE_TOP_Z = float(m.GRIP_POINT[2])  # the grip section's centre line, through the bar
HANDLE_FILLET = 0.006  # centre-line radius where the grip section turns down into a leg
HANDLE_WIDTH = m.BAR_SIZE[1]  # 24 mm, the bar's own width, all the way round
GRIP_THICK = m.BAR_SIZE[2]  # 10 mm, the bar's own thickness, along the grip section
LEG_THICK = m.POST_SIZE[0] + 0.001  # the legs enclose the 12 mm posts
FOOT_FLARE = 0.008  # extra thickness where a leg meets the body


def _corner_centres():
    """Centres of the rounded corners (half's frame, x and y) and the outline's edge normals."""
    tri = m.TRIANGLE
    centres, normals = [], []
    for i in range(3):
        a, b = tri[i], tri[(i + 1) % 3]
        edge = (b - a) / np.linalg.norm(b - a)
        normals.append(np.array((edge[1], -edge[0])))  # outward for a counter-clockwise outline
    for i in range(3):
        n_in, n_out = normals[i - 1], normals[i]  # the edges arriving at and leaving corner i
        bisector = -(n_in + n_out) / np.linalg.norm(n_in + n_out)
        half_angle = np.arccos(np.clip(-n_in @ n_out, -1., 1.)) / 2.  # half the corner's interior angle
        centres.append(tri[i] + bisector * CORNER_R / np.sin(half_angle))
    return np.array(centres), np.array(normals)


CENTRES, EDGE_NORMALS = _corner_centres()


def outline(inset):
    """The body's outline moved in by ``inset``: points (x, y) and outward unit normals, counter-clockwise.
    Every outline has the same number of points, so outlines can be joined into side walls."""
    points, normals = [], []
    for i in range(3):
        start = np.arctan2(*EDGE_NORMALS[i - 1][::-1])
        stop = np.arctan2(*EDGE_NORMALS[i][::-1])
        stop = stop if stop > start else stop + 2 * np.pi
        for t in np.linspace(start, stop, ARC_STEPS + 1):
            normal = np.array((np.cos(t), np.sin(t)))
            points.append(CENTRES[i] + (CORNER_R - inset) * normal)
            normals.append(normal)
    return np.array(points), np.array(normals)


def _rings_mesh(rings, top=None, bottom=None):
    """A closed-sided surface through rings of (points3, normals3), lowest first, plus optional flat caps
    at the first (bottom) and last (top) ring. Returns (points, normals, face counts, face indices) with
    one point per face corner, so each face carries its own normals."""
    points, normals, counts, indices = [], [], [], []

    def face(corners):
        # Wind each face so that its geometric normal agrees with the normals given for it.
        p = np.array([c[0] for c in corners])
        geometric = np.cross(p[1] - p[0], p[2] - p[0]) if len(p) == 4 or len(p) == 3 else np.zeros(3)
        if len(p) > 4:
            centre = p.mean(axis=0)
            geometric = sum(np.cross(p[i] - centre, p[(i + 1) % len(p)] - centre) for i in range(len(p)))
        if geometric @ sum(np.asarray(c[1]) for c in corners) < 0.:
            corners = corners[::-1]
        counts.append(len(corners))
        for p, n in corners:
            indices.append(len(points))
            points.append(p)
            normals.append(n)

    for (pa, na), (pb, nb) in zip(rings[:-1], rings[1:]):
        k = len(pa)
        for i in range(k):
            j = (i + 1) % k
            face(((pa[i], na[i]), (pa[j], na[j]), (pb[j], nb[j]), (pb[i], nb[i])))
    if top is not None:
        p = rings[-1][0]
        face([(q, np.array((0., 0., 1.))) for q in p])
    if bottom is not None:
        p = rings[0][0]
        face([(q, np.array((0., 0., -1.))) for q in p[::-1]])
    return np.array(points), np.array(normals), counts, indices


def body_mesh():
    """The half's shell, drawn: rounded corners, vertical sides, a rounded top edge, a flat back."""
    rings = []
    for z, inset, tilt in ([(RIM_LIFT, 0., 0.), (m.BODY_T - BEVEL, 0., 0.)]
                           + [(m.BODY_T - BEVEL + BEVEL * np.sin(a), BEVEL * (1 - np.cos(a)), a)
                              for a in np.linspace(0., np.pi / 2, BEVEL_STEPS + 1)[1:]]):
        xy, nxy = outline(inset)
        pts = np.c_[xy, np.full(len(xy), z)]
        nrm = np.c_[nxy * np.cos(tilt), np.full(len(xy), np.sin(tilt))]
        rings.append((pts, nrm))
    return _rings_mesh(rings, top=True, bottom=True)


def pad_mesh():
    """The cleaning pad, drawn as its own layer under the body and a little smaller than it."""
    xy, nxy = outline(PAD_INSET)
    rings = [(np.c_[xy, np.full(len(xy), z)], np.c_[nxy, np.zeros(len(xy))]) for z in (0., m.PAD_T + RIM_LIFT)]
    return _rings_mesh(rings, top=True, bottom=True)


def handle_path(steps=10):
    """Centre line of the D handle in the half's frame (x, z), from one foot over the grip to the other,
    with the section's thickness at each point."""
    r, x0, zt = HANDLE_FILLET, HANDLE_LEG_X, HANDLE_TOP_Z
    foot = m.BODY_T - 0.002  # the feet go 2 mm into the body
    pts, thick = [], []
    # Leg on the -x side, from the foot up to the fillet.
    for z in np.linspace(foot, zt - r, steps):
        pts.append((-x0, z))
        flare = max(0., 1. - (z - foot) / 0.010)
        thick.append(LEG_THICK + FOOT_FLARE * flare ** 2)
    for a in np.linspace(np.pi, np.pi / 2, steps)[1:]:  # fillet into the grip section
        pts.append((-x0 + r + r * np.cos(a), zt - r + r * np.sin(a)))
        thick.append(LEG_THICK + (GRIP_THICK - LEG_THICK) * (np.pi - a) / (np.pi / 2))
    for x in np.linspace(-x0 + r, x0 - r, 2 * steps)[1:-1]:  # the grip section, along the bar
        pts.append((x, zt))
        thick.append(GRIP_THICK)
    left = list(zip(pts, thick))
    right = [((-p[0], p[1]), t) for p, t in reversed(left)]
    mid = [((x0 - r, zt), GRIP_THICK)]
    path = left + mid + right
    return np.array([p for p, _ in path]), np.array([t for _, t in path])


def handle_mesh():
    """The D handle: a rounded-rectangle section swept along `handle_path`."""
    path, thick = handle_path()
    section = []  # rounded rectangle in (lateral y, normal) with unit normals, counter-clockwise
    rad = 0.003
    for cy, cn, a0 in ((1, 1, 0.), (-1, 1, np.pi / 2), (-1, -1, np.pi), (1, -1, 1.5 * np.pi)):
        for a in np.linspace(a0, a0 + np.pi / 2, 4):
            section.append((cy, cn, np.cos(a), np.sin(a)))
    rings = []
    for i, (p, t) in enumerate(zip(path, thick)):
        tangent = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        tangent /= np.linalg.norm(tangent)
        up = np.array((-tangent[1], tangent[0]))  # the section's normal axis, pointing out of the arch
        pts, nrm = [], []
        for cy, cn, ny, nn in section:
            y = cy * (HANDLE_WIDTH / 2 - rad) + ny * rad
            s = cn * (t / 2 - rad) + nn * rad
            q = p + s * up
            pts.append((q[0], y, q[1]))
            normal = nn * up
            nrm.append((normal[0], ny, normal[1]))
        rings.append((np.array(pts), np.array(nrm)))
    pts, nrm, counts, idx = _rings_mesh(rings)
    return pts, nrm, counts, idx


# -- the wall ------------------------------------------------------------------------------------
WALL_DEPTH = 0.070  # drawn depth of the wall outdoors of the board, so the wall reads about 80 mm thick
FRAME_DEPTH = 0.045  # the window frame lines the cut-out through the board and this far into the reveal
FRAME_PROUD = 0.001  # how far the frame lining stands into the opening
MULLION_U = (m.STILE_U[0] - 0.001, m.STILE_U[1] + 0.008)  # drawn over the stile, indoors only; 1 mm into the open light
MULLION_PROUD = 0.012  # how far it stands out from the board's indoor face
BEAD = 0.010  # glazing bead width along the glass
BEAD_END = 0.005  # narrower at the glass's far end, where the cleaner's corner passes
BEAD_PROUD = 0.006  # how far the beads stand off the glass
CASING_W = (0.040, 0.028)  # architrave width: beside the open light and above; beside the far end
CASING_PROUD = 0.012
BOARD_Z = 0.018  # interior window board thickness, under the cut-out
BOARD_PROJECTION = 0.025  # how far it stands out from the board's indoor face (short: the right wrist passes over it)
SKIRTING = (0.070, 0.012)  # skirting height and thickness
WALL_COLORS = {  # linear RGB
    "render": (0.62, 0.56, 0.48), "frame": (0.93, 0.93, 0.91), "trim": (0.90, 0.89, 0.86),
    "window_board": (0.88, 0.87, 0.84),
}
HALF_COLORS = {"pad_cloth": (0.42, 0.45, 0.50)}  # grey microfibre


def wall_shapes():
    """Every drawn box of the wall as name -> ((u0, u1), (n0, n1), (z0, z1), colour key), pane frame."""
    out_n, in_n = m.WALL_N
    lo, hi = m.OPENING_U[0], m.PANE_X[1]  # the whole cut-out, open light and fixed light
    z0, z1 = m.OPENING_Z
    depth = (out_n - WALL_DEPTH, out_n)
    frame_n = (out_n - FRAME_DEPTH, in_n)
    p, f = FRAME_PROUD, 0.004  # frame lining: 4 mm thick, standing 1 mm into the opening
    glass_in, glass_out = m.FACE_Y["inside"], m.FACE_Y["outside"]
    shapes = {
        # the wall's depth outdoors of the board, round the cut-out
        "depth_below": (m.WALL_U, depth, (0., z0), "render"),
        "depth_above": (m.WALL_U, depth, (z1, m.WALL_H), "render"),
        "depth_right": ((m.WALL_U[0], lo), depth, (z0, z1), "render"),
        "depth_left": ((hi, m.WALL_U[1]), depth, (z0, z1), "render"),
        # the window frame lining the cut-out (sill, head, the open light's jamb, the fixed light's jamb)
        "frame_sill": ((lo - f, hi + f), frame_n, (z0 - f, z0 + p), "frame"),
        "frame_head": ((lo - f, hi + f), frame_n, (z1 - p, z1 + f), "frame"),
        "frame_jamb_open": ((lo - f, lo + p), frame_n, (z0, z1), "frame"),
        "frame_jamb_fixed": ((hi - p, hi + f), frame_n, (z0, z1), "frame"),
        # the mullion over the stile, indoors
        "mullion": (MULLION_U, (out_n, in_n + MULLION_PROUD), (z0, z1), "frame"),
        # glazing beads round the fixed light, indoors and outdoors
        "bead_in_bottom": ((m.PANE_X[0], hi), (glass_in, glass_in + BEAD_PROUD), (z0, z0 + BEAD), "frame"),
        "bead_in_top": ((m.PANE_X[0], hi), (glass_in, glass_in + BEAD_PROUD), (z1 - BEAD, z1), "frame"),
        "bead_in_stile": ((m.PANE_X[0], m.PANE_X[0] + BEAD), (glass_in, glass_in + BEAD_PROUD), (z0, z1), "frame"),
        "bead_in_end": ((hi - BEAD_END, hi), (glass_in, glass_in + BEAD_PROUD), (z0, z1), "frame"),
        "bead_out_bottom": ((m.PANE_X[0], hi), (glass_out - BEAD_PROUD, glass_out), (z0, z0 + BEAD), "frame"),
        "bead_out_top": ((m.PANE_X[0], hi), (glass_out - BEAD_PROUD, glass_out), (z1 - BEAD, z1), "frame"),
        "bead_out_stile": ((m.PANE_X[0], m.PANE_X[0] + BEAD), (glass_out - BEAD_PROUD, glass_out), (z0, z1), "frame"),
        "bead_out_end": ((hi - BEAD_END, hi), (glass_out - BEAD_PROUD, glass_out), (z0, z1), "frame"),
        # architrave round the window, indoors
        "casing_open_side": ((lo - CASING_W[0], lo), (in_n, in_n + CASING_PROUD), (z0 - BOARD_Z, z1 + CASING_W[0]), "trim"),
        "casing_far_side": ((hi, hi + CASING_W[1]), (in_n, in_n + CASING_PROUD), (z0 - BOARD_Z, z1 + CASING_W[0]), "trim"),
        "casing_top": ((lo - CASING_W[0], hi + CASING_W[1]), (in_n, in_n + CASING_PROUD), (z1, z1 + CASING_W[0]), "trim"),
        # interior window board under the cut-out, its top level with the opening's bottom
        "window_board": ((lo - CASING_W[0] - 0.010, min(hi + CASING_W[1] + 0.002, m.WALL_U[1])),
                         (in_n, in_n + BOARD_PROJECTION), (z0 - BOARD_Z, z0), "window_board"),
        # skirting board along the foot of the wall, between the wall's feet
        "skirting": ((m.FOOT_U[0] + m.FOOT_SIZE[0] / 2, m.FOOT_U[1] - m.FOOT_SIZE[0] / 2),
                     (in_n, in_n + SKIRTING[1]), (0., SKIRTING[0]), "trim"),
    }
    return shapes


def mesh_text(prim, name, material, mesh, indent):
    """A USD Mesh with no physics: points, per-corner normals and faces."""
    pts, nrm, counts, idx = mesh
    pad = " " * indent
    f = lambda v: "(" + ", ".join(f"{float(x) + 0.:.6g}" for x in v) + ")"  # noqa: E731
    lines = [f'{pad}def Mesh "{name}" (', f'{pad}    prepend apiSchemas = ["MaterialBindingAPI"]', f"{pad})", f"{pad}{{",
             f"{pad}    rel material:binding = </Fixture/Materials/{material}>",
             f'{pad}    uniform token subdivisionScheme = "none"',
             f"{pad}    float3[] extent = [{f(pts.min(axis=0))}, {f(pts.max(axis=0))}]",
             f"{pad}    int[] faceVertexCounts = [{', '.join(map(str, counts))}]",
             f"{pad}    int[] faceVertexIndices = [{', '.join(map(str, idx))}]",
             f"{pad}    normal3f[] normals = [{', '.join(f(v) for v in nrm)}] (",
             f'{pad}        interpolation = "faceVarying"', f"{pad}    )",
             f"{pad}    point3f[] points = [{', '.join(f(v) for v in pts)}]",
             f"{pad}}}", ""]
    return "\n".join(lines) + "\n"
