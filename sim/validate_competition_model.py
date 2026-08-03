#!/usr/bin/env python3
"""无 GUI 验证生成的 MJCF、夹爪、物体和 raycaster 插件。"""
import argparse
from pathlib import Path
import mujoco


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-root", type=Path,
                        default=Path("/workspace/.runtime/raicom2026/sim_mujoco"))
    args = parser.parse_args()
    plugin = args.sim_root / "bin" / "mujoco_plugin" / "libsensor_ray.so"
    mujoco.mj_loadPluginLibrary(str(plugin))
    model_path = (args.sim_root / "configuration" / "robot" /
                  "lx2501_3_t2d5_raicom" / "model_info" / "scene.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for _ in range(20):
        mujoco.mj_step(model, data)
    required_joints = ("L_claw_joint", "R_claw_joint")
    required_bodies = ("medicine_box", "bread")
    for name in required_joints:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            raise RuntimeError(f"缺少关节 {name}")
    for name in required_bodies:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            raise RuntimeError(f"缺少物体 {name}")
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "raycaster_lidar")
    if sensor_id < 0:
        raise RuntimeError("缺少 raycaster_lidar")
    plugin_id = model.sensor_plugin[sensor_id]
    state = model.plugin_stateadr[plugin_id]
    horizontal, vertical = int(data.plugin_state[state]), int(data.plugin_state[state + 1])
    print(f"model_ok joints={required_joints} bodies={required_bodies}")
    print(f"lidar_rays={horizontal}x{vertical} sensor_dim={model.sensor_dim[sensor_id]}")
    if horizontal <= 0 or vertical <= 0:
        raise RuntimeError("raycaster 插件未产生有效射线")


if __name__ == "__main__":
    main()
