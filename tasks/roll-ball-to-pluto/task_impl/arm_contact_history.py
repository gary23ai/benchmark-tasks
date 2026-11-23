"""Latch a forbidden arm contact measured at any native physics substep."""

import numpy as np


def reporting_links(arm):
    from yam_arena.robot import asset_link_paths

    prefix = '{ENV_REGEX_NS}/' + arm.removesuffix('_tcp').title() + 'Arm'
    return [prefix + suffix for suffix in asset_link_paths().values()]


def configure(config, guards):
    from isaaclab.sensors.contact_sensor.contact_sensor_cfg import ContactSensorCfg

    actors = '{ENV_REGEX_NS}/task_fixture/Actors/'
    for guard in guards:
        filters = [actors + name for name in guard['targets']]
        for number, path in enumerate(reporting_links(guard['a'])):
            setattr(config.scene, f"forbidden_{guard['name']}_{number}", ContactSensorCfg(
                prim_path=path, filter_prim_paths_expr=filters,
                update_period=0., history_length=config.decimation,
            ))
    return config


class ArmContactHistory:
    def __init__(self, config, scene, guards):
        self.channels = {
            guard['name']: (len(guard['targets']), [scene[f"forbidden_{guard['name']}_{number}"]
                for number, _ in enumerate(reporting_links(guard['a']))])
            for guard in guards
        }
        self.substeps = config.decimation
        self.reset()

    def reset(self):
        self.touched = set()

    def observe(self):
        for name, (target_count, sensors) in self.channels.items():
            if name in self.touched:
                continue
            touched = False
            for sensor in sensors:
                history = sensor.data.normal_force_matrix_w_history
                expected = (1, self.substeps, 1, target_count, 3)
                if history is None or tuple(history.torch.shape) != expected:
                    raise RuntimeError('Missing forbidden-pair physics-step history for ' + name)
                force = history.torch[0, :, 0].detach().cpu().numpy()
                if not np.isfinite(force).all():
                    raise RuntimeError('Invalid physics-step force history for ' + name)
                touched |= bool(np.any(np.linalg.norm(force, axis=-1) > 1e-8))
            if touched:
                self.touched.add(name)

    def measure(self, name):
        return float(name in self.touched)
