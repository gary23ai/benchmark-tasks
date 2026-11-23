"""Retain the first loaded pad across the native physics-step history."""

import numpy as np


def configure_impact_sensors(config, metrics):
    from isaaclab.sensors.contact_sensor.contact_sensor_cfg import ContactSensorCfg

    root = "{ENV_REGEX_NS}/task_fixture/Actors/"
    for metric in metrics:
        setattr(config.scene, "impact_" + metric["name"], ContactSensorCfg(
            prim_path=root + metric["a"], filter_prim_paths_expr=[root + metric["b"]],
            update_period=0.0, history_length=config.decimation,
        ))
    return config


class FirstImpact:
    def __init__(self, config, scene, metrics):
        self.substeps = config.decimation
        self.sensors = {m["name"]: scene["impact_" + m["name"]] for m in metrics}
        self.first = frozenset()

    def reset(self):
        self.first = frozenset()

    def observe(self):
        if self.first:
            return
        signals = {}
        for name, sensor in self.sensors.items():
            history = sensor.data.normal_force_matrix_w_history
            if history is None or tuple(history.torch.shape) != (1, self.substeps, 1, 1, 3):
                raise RuntimeError("Missing per-pad physics-step contact history")
            forces = history.torch[0, :, 0, 0].detach().cpu().numpy()
            if not np.isfinite(forces).all():
                raise RuntimeError("Invalid physics-step contact history")
            signals[name] = np.linalg.norm(forces, axis=1) > 1e-8
        # PhysX buffers are newest-first. All pads in one substep are simultaneous.
        for sample in range(self.substeps - 1, -1, -1):
            hits = frozenset(name for name, active in signals.items() if active[sample])
            if hits:
                self.first = hits
                return

    def measure(self, name):
        return float(name in self.first)
