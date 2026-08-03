#!/usr/bin/env python3
"""从官方只读模型生成国赛仿真覆盖模型，不修改官方目录。"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

SOURCE_NAME = "lx2501_3_t2d5"
TARGET_NAME = "lx2501_3_t2d5_raicom"


def child_by_name(root, tag, name):
    for element in root.iter(tag):
        if element.get("name") == name:
            return element
    raise RuntimeError(f"MJCF 中找不到 {tag} {name}")


def add_gripper(root, side: str):
    wrist = child_by_name(root, "body", f"{side}_wrist_roll_link")
    sign = 1 if side == "left" else -1
    # Match the bundled official IK URDF's OmniPicker base transform.
    base = ET.SubElement(wrist, "body", name=f"{side}_omnipicker_base",
                         pos="0 0 -0.046", euler="3.14159 0 3.14159")
    ET.SubElement(base, "geom", name=f"{side}_gripper_palm", type="box",
                  pos="0 0 0.025", size="0.035 0.055 0.018", mass="0.01",
                  rgba="0.18 0.18 0.20 1")
    internal = "L_claw_joint" if side == "left" else "R_claw_joint"
    mirror_joint = f"{side}_claw_mirror_joint"
    for label, joint_name, y, axis in (("claw", internal, -0.045, "0 -1 0"),
                                       ("claw_mirror", mirror_joint, 0.045, "0 1 0")):
        finger = ET.SubElement(base, "body", name=f"{side}_{label}_link",
                               pos=f"0 {y} 0.05")
        ET.SubElement(finger, "joint", name=joint_name, type="slide",
                      axis=axis, range="0 0.04", damping="2", frictionloss="1")
        ET.SubElement(finger, "geom", name=f"{side}_{label}_finger", type="box",
                      pos="0 0 0.08", size="0.014 0.012 0.07", mass="0.01",
                      friction="2 0.02 0.002", rgba="0.08 0.08 0.09 1")

    actuator = root.find("actuator")
    # 官方模拟器对 active joint 自己做位置闭环，MJCF 端必须与其余关节一样是力矩 motor。
    ET.SubElement(actuator, "motor", name=f"motor_{internal}",
                  joint=internal, ctrlrange="-20 20")
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "joint", name=f"{side}_claw_symmetry",
                  joint1=f"{side}_claw_joint", joint2=f"{side}_claw_mirror_joint",
                  polycoef="0 1 0 0 0")
    sensor = root.find("sensor")
    # 闭源 hand subscriber 会把 ROS 的 left/right_claw_joint 映射到 L/R_claw_joint。
    equality[-1].set("joint1", internal)
    for joint in (internal, mirror_joint):
        ET.SubElement(sensor, "jointpos", name=f"jointpos_{joint}", joint=joint)
        ET.SubElement(sensor, "jointvel", name=f"jointvel_{joint}", joint=joint)


def add_lidar(root):
    extension = ET.Element("extension")
    ET.SubElement(extension, "plugin", plugin="mujoco.sensor.ray_caster_lidar")
    root.insert(0, extension)
    pelvis = child_by_name(root, "body", "pelvis")
    # 控制器稳定后骨盆约高 0.66 m。宽垂直视场同时向下扫描低墙、
    # 向前扫描桌面物品；导航与物品定位共用这一真实射线点云。
    ET.SubElement(pelvis, "camera", name="RayCasterLidar", pos="0.10 0 0",
                  xyaxes="0 -1 0 0 0 1", mode="fixed")
    sensor = root.find("sensor")
    plugin = ET.SubElement(sensor, "plugin", name="raycaster_lidar",
                           plugin="mujoco.sensor.ray_caster_lidar",
                           objtype="camera", objname="RayCasterLidar")
    # Keep ray casting cheap enough for the closed-loop MC and MuJoCo physics
    # to remain synchronized. 90x5 at 5 Hz still gives 4-degree horizontal
    # coverage for the 5 cm occupancy grid; denser scans made simulated time
    # run far behind wall time and destabilized the real-time gait controller.
    for key, value in (("fov_h", "270"), ("fov_v", "60"), ("size", "90 5"),
                       ("dis_range", "0.08 8"), ("n_step_update", "200"),
                       # 项目雷达直接输出 map/world 坐标命中点。
                       ("sensor_data_types", "pos_w data")):
        ET.SubElement(plugin, "config", key=key, value=value)


def add_service_objects(scene_root):
    world = scene_root.find("worldbody")
    # Three randomized-service stand-ins sit along the reachable front edge of
    # the official table.  Their different height/shape supports lidar-based
    # identification; no object is attached or scripted.
    mug_frame = child_by_name(scene_root, "frame", "mug_frame")
    # The table's front edge is x=1.222 m. Keep each body fully supported but
    # close enough for the stock X2 arm to reach at tabletop height without
    # driving the torso into the table.
    mug_frame.set("pos", "1.27 -1.40 0.55")
    medicine = ET.SubElement(world, "body", name="medicine_box", pos="1.27 -1.60 0.59")
    ET.SubElement(medicine, "freejoint")
    ET.SubElement(medicine, "geom", type="box", size="0.045 0.03 0.07", mass="0.08",
                  friction="1.5 0.01 0.001", rgba="0.95 0.2 0.2 1")
    bread = ET.SubElement(world, "body", name="bread", pos="1.27 -1.20 0.56")
    ET.SubElement(bread, "freejoint")
    ET.SubElement(bread, "geom", type="ellipsoid", size="0.065 0.035 0.035", mass="0.06",
                  friction="1.5 0.01 0.001", rgba="0.82 0.55 0.20 1")


def update_robot_yaml(path: Path, *, grippers: bool = True, lidar: bool = True):
    data = yaml.safe_load(path.read_text())
    # The closed simulator's hand subscriber does not drive custom actuators.
    # Put the physical claws in its generic arm position loop, while default.yaml
    # redirects the resulting 16-axis state to a private topic.  A relay exposes
    # exactly the official 14 axes to MC, preserving its strict input dimension.
    if grippers:
        arm_index = data["actual"]["active_joint_frame_type"].index("arm")
        data["actual"]["active_joint_name"][arm_index].extend(
            ["L_claw_joint", "R_claw_joint"])
        data["actual"]["active_joint_control_type"][arm_index].extend(
            ["position", "position"])
        data["logical"]["nominal_configuration"][arm_index].extend([0.04, 0.04])
    if lidar:
        data["sensor"]["sensor_frame_type"].append("raycaster_lidar")
        data["sensor"]["sensor_name"].append(["raycaster_lidar"])
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def update_sim_yaml(path: Path, *, grippers: bool = True, lidar: bool = True):
    data = yaml.safe_load(path.read_text())
    aimrt = data["aimrt"]
    pub = aimrt["channel"]["pub_topics_options"]
    sub = aimrt["channel"]["sub_topics_options"]
    if grippers:
        for option in pub:
            if option.get("topic_name") == "/aima/hal/joint/arm/state":
                option["topic_name"] = "/aima/sim/joint/arm/state_raw"
    if grippers:
        pub.append({"topic_name": "/aima/hal/joint/hand/state", "enable_backends": ["ros2"]})
        sub.append({"topic_name": "/aima/hal/joint/hand/command", "enable_backends": ["ros2"]})
    if lidar:
        pub.append({"topic_name": "/aima/sim/lidar/points", "enable_backends": ["ros2"]})
    manager = data["MujocoSimModule"]
    if grippers:
        manager["SubscriberManager"]["subscriber_cfg_list"].append({
            "name": "hand_joint_command", "topic": "/aima/hal/joint/hand/command",
            "enable": True, "executor": "joint_command_subscriber_thread",
            "wait_ready": False, "print_interval": 1})
    publishers = manager["PublisherManager"]["publisher_cfg_list"]
    if grippers:
        for publisher in publishers:
            if publisher.get("name") == "arm_joint_state":
                publisher["topic"] = "/aima/sim/joint/arm/state_raw"
                publisher["frequency"] = 500
    if grippers:
        publishers.append(
            {"name": "hand_joint_state", "topic": "/aima/hal/joint/hand/state", "enable": True,
             "executor": "common_joint_state_publisher_thread", "frequency": 100,
             "print_interval": 0})
    if lidar:
        publishers.append(
            {"name": "raycaster_lidar_ros", "topic": "/aima/sim/lidar/points", "enable": True,
             "executor": "data_state_publisher_thread", "frequency": 20,
             "print_interval": 0})
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def generate(configuration_root: Path, *, grippers: bool = True, lidar: bool = True) -> Path:
    source, target = configuration_root / "robot" / SOURCE_NAME, configuration_root / "robot" / TARGET_NAME
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    model = target / "model_info" / "x2.xml"
    tree = ET.parse(model)
    root = tree.getroot()
    if grippers:
        add_gripper(root, "left")
        add_gripper(root, "right")
    if lidar:
        add_lidar(root)
    ET.indent(tree, space="  ")
    tree.write(model, encoding="unicode")
    scene = target / "model_info" / "scene.xml"
    scene_tree = ET.parse(scene)
    add_service_objects(scene_tree.getroot())
    ET.indent(scene_tree, space="  ")
    scene_tree.write(scene, encoding="unicode")
    update_robot_yaml(target / "model_info" / "default.yaml",
                      grippers=grippers, lidar=lidar)
    update_sim_yaml(target / "simulator" / "default.yaml",
                    grippers=grippers, lidar=lidar)
    (target / "GENERATED_BY_RAICOM.txt").write_text(
        "Generated by /workspace/sim/prepare_competition_assets.py; official source untouched.\n")
    return target


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--configuration-root", type=Path,
                        default=Path("/workspace/.runtime/raicom2026/sim_mujoco/configuration"))
    parser.add_argument("--without-grippers", action="store_true",
                        help="diagnostic: generate lidar-only overlay")
    parser.add_argument("--without-lidar", action="store_true",
                        help="diagnostic: generate gripper-only overlay")
    args = parser.parse_args()
    print(generate(args.configuration_root,
                   grippers=not args.without_grippers,
                   lidar=not args.without_lidar))


if __name__ == "__main__":
    main()
