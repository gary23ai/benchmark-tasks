"""A cold pan, two ingredients, a utensil crock and two joint-driven YAM arms."""

from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
PAN = np.array([0.36, 0, 0.004])
CROCK = np.array([0.28, 0.16, 0])
GRIP_Z = 0.145
LIMIT = 540
FOODS = tuple(f"{kind}{i}" for kind in ("carrot", "potato") for i in range(6))
READY = np.array([0.0, 0.63, 0.74, -0.89, 0.0, 0.0])
SIDES = ("left", "right")


def fmt(v):
    return " ".join(f"{x:.10g}" for x in v)


def add(parent, tag, **attrs):
    return ET.SubElement(
        parent,
        tag,
        {k: (v if isinstance(v, str) else fmt(v)) for k, v in attrs.items()},
    )


def geom(body, name, typ, pos, size, rgb, **attrs):
    return add(
        body, "geom", name=name, type=typ, pos=pos, size=size, rgba=(*rgb, 1), **attrs
    )


def arm_models(root, world, actuators):
    source = ET.parse(HERE / "yam/yam_pro_linear_4310.xml").getroot()
    assets = root.find("asset")
    for mesh in source.find("asset"):
        el = deepcopy(mesh)
        el.set("file", str(HERE / "yam" / mesh.get("file")))
        assets.append(el)
    stock = mj.MjModel.from_xml_path(str(HERE / "yam/yam_pro_linear_4310.xml"))
    d = mj.MjData(stock)
    mj.mj_forward(stock, d)
    tcp = stock.site("grasp_site").id
    for side, y, bit, other in (("left", 0.305, 2, 4), ("right", -0.305, 4, 2)):
        body = deepcopy(source.find("worldbody/body"))
        body.set("pos", fmt([0, y, 0]))
        for obj in body.iter():
            if obj.get("name"):
                obj.set("name", side + "_" + obj.get("name"))
            if obj.tag == "body":
                obj.set("gravcomp", "1")
            if obj.tag == "geom":
                obj.set("contype", str(bit))
                obj.set("conaffinity", str(other | 1))
                if obj.get("mesh") in ("tip_left", "tip_right"):
                    obj.set("contype", "0")
                    obj.set("conaffinity", "0")
                obj.set(
                    "rgba",
                    ".23 .30 .34 1"
                    if obj.get("mesh") not in ("gripper", "tip_left", "tip_right")
                    else ".12 .14 .15 1",
                )
            if obj.tag == "joint":
                obj.set("damping", "1.2" if obj.get("type") != "slide" else "12")
                obj.set("armature", ".025" if obj.get("type") != "slide" else ".03")
            if obj.tag == "site":
                obj.set("group", "5")
        # Pads expressed in the grasp-site frame, converted to each moving finger.
        for finger, sign in (("tip_left", 1), ("tip_right", -1)):
            idx = stock.body(finger).id
            rot = d.xmat[idx].reshape(3, 3).T
            sr = d.site_xmat[tcp].reshape(3, 3)
            local = rot @ (
                d.site_xpos[tcp] + sr @ [0, 0.003 * sign, -0.009] - d.xpos[idx]
            )
            quat = Rotation.from_matrix(rot @ sr).as_quat(scalar_first=True)
            parent = body.find(f".//body[@name='{side}_{finger}']")
            geom(
                parent,
                side + "_" + finger + "_pad",
                "box",
                local,
                [0.014, 0.003, 0.020],
                [0.17, 0.17, 0.16],
                quat=quat,
                mass="0",
                friction="1.0 .005 .0002",
                condim="4",
                priority="1",
                contype=str(bit),
                conaffinity=str(other | 1),
            )
        wrist = body.find(f".//body[@name='{side}_link6']")
        flip = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
        camera_q = Rotation.from_matrix(
            flip
            @ Rotation.from_quat(
                [-0.003227, 0.002817, 0.975619, 0.21943], scalar_first=True
            ).as_matrix()
        ).as_quat(scalar_first=True)
        add(
            wrist,
            "camera",
            name=side,
            pos=flip @ [0, 0.069638, 0.072],
            quat=camera_q,
            fovy="63.2",
        )
        world.append(body)
        add(
            root.find("equality"),
            "joint",
            joint1=side + "_joint7",
            joint2=side + "_joint8",
            polycoef="0 1 0 0 0",
            solref=".004 1",
        )
        for idx, gain in enumerate([120, 160, 150, 48, 45, 40, 3500], 1):
            joint = side + "_joint" + str(idx)
            force = 50 if idx == 7 else 10 if idx >= 4 else 28
            j = body.find(f".//joint[@name='{joint}']")
            j.set("actuatorfrcrange", fmt([-force, force]))
            add(
                actuators,
                "position",
                name=joint,
                joint=joint,
                kp=str(gain),
                kv=str([9, 12, 10, 3, 3, 2, 20][idx - 1]),
                ctrllimited="true",
                ctrlrange=j.get("range"),
                forcerange=fmt([-force, force]),
            )


def build():
    root = ET.Element("mujoco", model="mix a cold pan of diced vegetables")
    add(root, "compiler", angle="degree")
    # The licensed arm model expresses its joint ranges in radians.
    root.find("compiler").set("angle", "radian")
    add(
        root,
        "option",
        timestep=".002",
        integrator="implicitfast",
        iterations="55",
        cone="elliptic",
    )
    vis = add(root, "visual")
    add(vis, "global", offwidth="800", offheight="600")
    add(
        vis,
        "headlight",
        ambient=".38 .38 .38",
        diffuse=".43 .43 .43",
        specular=".10 .10 .10",
    )
    assets = add(root, "asset")
    add(
        assets,
        "texture",
        name="sky",
        type="skybox",
        builtin="gradient",
        rgb1=".52 .59 .62",
        rgb2=".84 .83 .78",
        width="64",
        height="256",
    )
    world = add(root, "worldbody")
    add(root, "equality")
    actuators = add(root, "actuator")
    arm_models(root, world, actuators)
    add(
        world,
        "light",
        pos=".3 -.4 1.4",
        dir="0 .15 -1",
        directional="true",
        diffuse=".55 .55 .55",
        specular=".12 .12 .12",
    )
    geom(
        world,
        "table",
        "box",
        [0.24, 0, -0.030],
        [0.52, 0.54, 0.030],
        [0.57, 0.51, 0.42],
        friction=".8 .006 .0001",
    )
    add(
        world,
        "camera",
        name="top",
        pos=[-0.1665, -0.009, 0.9443],
        quat=[0.68301, 0.18301, -0.18301, -0.68301],
        fovy="63.0",
    )
    for name, eye, target, fov in [
        ("overview", [0.99, 0.27, 0.71], [0.22, 0.01, 0.12], 48),
        ("pan_view", [0.54, -0.11, 0.56], [0.358, 0, 0.025], 26),
    ]:
        back = np.subtract(eye, target)
        back /= np.linalg.norm(back)
        right = np.cross([0, 0, 1], back)
        right /= np.linalg.norm(right)
        add(
            world,
            "camera",
            name=name,
            pos=eye,
            xyaxes=fmt(np.r_[right, np.cross(back, right)]),
            fovy=str(fov),
        )
    for entry in json.loads((HERE / "pan.json").read_text()):
        position = np.array(entry["position"]) - [0, 0, 0.012]
        if entry["name"] == "pan":
            position = PAN
        body = add(world, "body", name=entry["name"], pos=position)
        if entry["name"] == "pan":
            body.set("euler", fmt([0, 0, np.pi / 2]))
            add(body, "freejoint", name="pan")
            add(
                body,
                "inertial",
                pos="-.020 0 .025",
                mass=".56",
                diaginertia=".0015 .0033 .0036",
            )
        for i, p in enumerate(entry["parts"]):
            typ = p["type"]
            w, h, z = p["size"]
            size = (
                [w / 2, h / 2, z / 2]
                if typ == "box"
                else [w / 2, (z - (w if typ == "capsule" else 0)) / 2]
            )
            color = (
                entry["rgb"] if entry["name"] != "pan" or i < 17 else [0.15, 0.14, 0.13]
            )
            geom(
                body,
                entry["name"] + str(i),
                typ,
                p["offset"],
                size,
                color,
                euler=np.radians(p["euler_deg"]),
                friction=f"{entry['friction']} .003 .0001",
            )
        if entry["name"] == "pan":
            # A coplanar inner square gives the thin flat cylinder a stable support manifold.
            base = geom(
                body,
                "flat_base",
                "box",
                [0, 0, 0.002],
                [0.054, 0.054, 0.002],
                entry["rgb"],
                mass="0",
            )
            base.set("rgba", "0 0 0 0")
    # A square ceramic utensil crock with a flat bottom, open at the top.
    crock = add(world, "body", name="crock", pos=CROCK)
    glaze = [0.42, 0.59, 0.57]
    geom(crock, "crock_base", "box", [0, 0, 0.003], [0.031, 0.031, 0.003], glaze)
    for i, (x, y, sx, sy) in enumerate(
        [
            (-0.029, 0, 0.002, 0.031),
            (0.029, 0, 0.002, 0.031),
            (0, -0.029, 0.027, 0.002),
            (0, 0.029, 0.027, 0.002),
        ]
    ):
        geom(crock, f"crock{i}", "box", [x, y, 0.035], [sx, sy, 0.035], glaze)
    # Two strips in the base leave a straight 8 mm slot. The tool lifts freely.
    for sign in (-1, 1):
        geom(
            crock,
            f"insert_{sign}",
            "box",
            [sign * 0.0155, 0, 0.014],
            [0.0115, 0.026, 0.008],
            glaze,
        )
    tool = add(world, "body", name="spatula", pos=CROCK + [0, 0, 0.0065])
    add(tool, "freejoint", name="tool")
    geom(
        tool,
        "blade",
        "box",
        [0, 0, 0.020],
        [0.002, 0.024, 0.020],
        [0.22, 0.43, 0.37],
        mass=".014",
        friction=".7 .004 .0001",
    )
    geom(
        tool,
        "neck",
        "capsule",
        [0, 0, 0.075],
        [0.004, 0.035],
        [0.30, 0.47, 0.40],
        mass=".010",
    )
    geom(
        tool,
        "handle",
        "box",
        [0, 0, 0.149],
        [0.006, 0.009, 0.045],
        [0.65, 0.43, 0.24],
        mass=".028",
        friction=".9 .004 .0001",
    )
    for i, name in enumerate(FOODS):
        row, col = divmod(i % 6, 3)
        if i < 6:
            x, y = -0.051 + 0.013 * col, (-0.009, 0.009)[row]
        else:
            x, y = 0.010 + 0.013 * col, (-0.020, 0.020)[row]
        b = add(
            world,
            "body",
            name=name,
            pos=PAN + [x, y, 0.012],
            euler=[0, 0, [0.09, -0.21, 0.28, 0.16, -0.1, 0.3][i % 6]],
        )
        add(b, "freejoint", name=name)
        geom(
            b,
            name,
            "box",
            [0, 0, 0],
            [0.005, 0.005, 0.005],
            ([0.91, 0.30, 0.055] if i < 6 else [0.87, 0.72, 0.42]),
            mass=".00105",
            friction=".43 .002 .0001",
        )
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
