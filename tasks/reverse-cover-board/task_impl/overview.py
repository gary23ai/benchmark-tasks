"""Measure whether the robot screens a board landmark in the fixed overview."""
import numpy as np


def configure_scene_queries(configure):
    def apply(cfg):
        cfg = configure(cfg)
        cfg.sim.enable_scene_query_support = True
        camera = cfg.scene.top_cam
        channel = "instance_id_segmentation_fast"
        camera.data_types = [*camera.data_types, channel]
        camera.renderer_cfg = camera.renderer_cfg.replace(colorize_instance_id_segmentation=False)
        return cfg
    return apply


def check_fixture_query(center):
    import carb
    from omni.physx import get_physx_scene_query_interface
    hits = []

    def collect(result):
        hits.append(str(result.collision))
        return True

    origin = np.asarray(center) + [0., 0., .5]
    get_physx_scene_query_interface().raycast_all(
        carb.Float3(*origin), carb.Float3(0., 0., -1.), 1.1, collect, True)
    if not any("/Actors/" in path for path in hits):
        raise RuntimeError("Camera inspection could not query any physical fixture collider")


def robot_mask(sensor):
    # The stock policy still observes only RGB and joints. The scorer reads the
    # visible render meshes, including dark housings that do not share a collider.
    channel = "instance_id_segmentation_fast"
    data = sensor.data
    output = data.output[channel]
    tensor = output.torch if hasattr(output, "torch") else output
    ids = tensor[0, ..., 0].detach().cpu().numpy()
    labels = (data.info or {}).get(channel, {}).get("idToLabels", {})
    robot = [int(i) for i, path in labels.items() if "/LeftArm/" in str(path) or "/RightArm/" in str(path)]
    if not robot or not any("/Actors/" in str(path) for path in labels.values()):
        raise RuntimeError("Overview instance IDs lack robot or fixture paths: " + str(list(labels.values())[:10]))
    return np.isin(ids, robot)


def arm_clear_fraction(camera, point_world, mask):
    from yam_arena.urdf import quat_xyzw_to_matrix

    if camera.convention != "opengl":
        raise ValueError("The inspection requires the station's OpenGL overview")
    height, width = mask.shape
    lens = camera.lens.resized(width, height)
    basis = quat_xyzw_to_matrix(np.asarray(camera.rot_xyzw))
    origin = np.asarray(camera.pos)
    local = basis.T @ (point_world - origin)
    if local[2] >= -0.01:
        return 0.
    u = lens.cx - lens.fx * local[0] / local[2]
    v = lens.cy + lens.fy * local[1] / local[2]
    u,v = int(round(u)),int(round(v))
    dx,dy = max(1, round(width/320)),max(1, round(height/176))
    if not (dx <= u < width - dx and dy <= v < height - dy) or mask[v,u]:
        return 0.
    neighbours = [mask[v,u+dx],mask[v,u-dx],mask[v+dy,u],mask[v-dy,u]]
    return (5 - np.count_nonzero(neighbours)) / 5.
