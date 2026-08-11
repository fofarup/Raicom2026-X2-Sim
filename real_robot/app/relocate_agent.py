#!/usr/bin/env python3
"""Guarded X2 SLAM relocalization for the real robot.

``check`` only inspects the map database and ROS graph.  ``execute`` publishes
the official relocalization command and initial pixel pose, then requires
several finite ``/slam/lidar_odom`` samples before reporting success.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sqlite3
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_MAP_DATABASE = Path("/agibot/data/var/MapManagerModule/map.db")
COMMAND_TOPIC = "/integrated_command"
INITIAL_POSE_TOPIC = "/relocalization_pose"


class RelocationError(RuntimeError):
    pass


@dataclass(frozen=True)
class MapMetadata:
    map_id: str
    map_name: str
    width: float
    height: float
    resolution_px_per_m: float
    origin_u: float
    origin_v: float


def _finite_number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RelocationError(f"地图字段不是数字: {label}={value!r}") from exc
    if not math.isfinite(number):
        raise RelocationError(f"地图字段不是有限数值: {label}={number}")
    return number


def validate_map_id(map_id: str) -> str:
    value = str(map_id).strip()
    if not value.isdigit() or int(value) <= 0:
        raise RelocationError(f"map_id 必须为正整数，实际为 {map_id!r}")
    return value


def load_map_metadata(database: Path, map_id: str) -> MapMetadata:
    map_id = validate_map_id(map_id)
    database = database.expanduser().resolve()
    if not database.is_file():
        raise RelocationError(f"地图数据库不存在: {database}")
    try:
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            row = connection.execute(
                "SELECT CAST(map_id AS TEXT), map_name, map_info "
                "FROM map WHERE CAST(map_id AS TEXT) = ?",
                (map_id,),
            ).fetchone()
        finally:
            connection.close()
    except sqlite3.Error as exc:
        raise RelocationError(f"只读查询地图数据库失败: {exc}") from exc
    if row is None:
        raise RelocationError(f"机器人地图数据库中不存在 map_id={map_id}")
    try:
        info = json.loads(row[2])
        origin = info["origin"]
    except (TypeError, KeyError, json.JSONDecodeError) as exc:
        raise RelocationError(f"map_id={map_id} 的 map_info 格式无效") from exc
    metadata = MapMetadata(
        map_id=str(row[0]),
        map_name=str(row[1]),
        width=_finite_number(info.get("width"), "width"),
        height=_finite_number(info.get("height"), "height"),
        resolution_px_per_m=_finite_number(info.get("resolution"), "resolution"),
        origin_u=_finite_number(origin.get("u"), "origin.u"),
        origin_v=_finite_number(origin.get("v"), "origin.v"),
    )
    if metadata.width <= 0 or metadata.height <= 0 or metadata.resolution_px_per_m <= 0:
        raise RelocationError("地图宽、高和分辨率必须为正数")
    return metadata


def validate_initial_pose(
    metadata: MapMetadata,
    pixel_x: float,
    pixel_y: float,
    yaw_deg: float,
    *,
    allow_non_origin: bool,
    origin_tolerance_px: float,
) -> tuple[float, float, float]:
    pixel_x = _finite_number(pixel_x, "pixel_x")
    pixel_y = _finite_number(pixel_y, "pixel_y")
    yaw_deg = _finite_number(yaw_deg, "yaw_deg")
    if not 0 <= pixel_x < metadata.width or not 0 <= pixel_y < metadata.height:
        raise RelocationError(
            f"初始像素位置 ({pixel_x}, {pixel_y}) 超出地图 "
            f"{metadata.width:g}x{metadata.height:g}"
        )
    origin_tolerance_px = _finite_number(
        origin_tolerance_px, "origin_tolerance_px"
    )
    if origin_tolerance_px < 0:
        raise RelocationError("origin_tolerance_px 不能为负数")
    origin_error = math.hypot(
        pixel_x - metadata.origin_u, pixel_y - metadata.origin_v
    )
    if origin_error > origin_tolerance_px and not allow_non_origin:
        raise RelocationError(
            f"初始位置距建图原点 {origin_error:.2f}px；若现场确认机器人不在原点，"
            "必须显式添加 --allow-non-origin"
        )
    normalized_yaw = (yaw_deg + 180.0) % 360.0 - 180.0
    return pixel_x, pixel_y, normalized_yaw


def yaw_quaternion(yaw_deg: float) -> tuple[float, float]:
    half_yaw = math.radians(yaw_deg) / 2.0
    return math.sin(half_yaw), math.cos(half_yaw)


def print_map(metadata: MapMetadata) -> None:
    print(
        f"MAP name={metadata.map_name} id={metadata.map_id} "
        f"size={metadata.width:g}x{metadata.height:g} "
        f"resolution={metadata.resolution_px_per_m:g}px/m "
        f"origin=({metadata.origin_u:g},{metadata.origin_v:g})",
        flush=True,
    )


def require_execution_guards(args: argparse.Namespace) -> None:
    if os.environ.get("RAICOM_CONFIRM_REAL_ROBOT") != "YES":
        raise RelocationError(
            "拒绝发布：请在确认真机和急停人员后设置 RAICOM_CONFIRM_REAL_ROBOT=YES"
        )
    if os.environ.get("RAICOM_CONFIRM_RELOCALIZATION") != "YES":
        raise RelocationError(
            "拒绝发布：请核对地图和实际站位后设置 RAICOM_CONFIRM_RELOCALIZATION=YES"
        )
    if not args.confirm_at_pose:
        raise RelocationError(
            "拒绝发布：必须用 --confirm-at-pose 确认机器人就在输入的像素位置和朝向"
        )


def run_ros(args: argparse.Namespace, metadata: MapMetadata) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        from std_msgs.msg import String
    except ImportError as exc:
        raise RelocationError(f"ROS 2 Python 环境未加载: {exc}") from exc

    try:
        from robot_profile import load_robot_profile

        profile = load_robot_profile()
    except Exception as exc:
        raise RelocationError(f"真机配置加载失败: {exc}") from exc
    localization_topic = str(profile["topics"]["localization_pose"])
    localization_type = str(profile["topics"]["localization_pose_type"])
    if localization_type != "nav_msgs/msg/Odometry":
        raise RelocationError(
            "重定位验收只支持 nav_msgs/msg/Odometry，"
            f"配置实际为 {localization_type}"
        )
    if localization_topic != "/slam/lidar_odom":
        raise RelocationError(
            "二号场地必须使用官方 /slam/lidar_odom，"
            f"配置实际为 {localization_topic}"
        )

    command_qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    localization_qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    class RelocationNode(Node):
        def __init__(self) -> None:
            super().__init__("raicom_relocation")
            self.command_pub = self.create_publisher(
                String, COMMAND_TOPIC, command_qos
            )
            self.pose_pub = self.create_publisher(Pose, INITIAL_POSE_TOPIC, 10)
            self.accept_samples = False
            self.samples: list[tuple[float, float, float]] = []
            self.localization_sub = self.create_subscription(
                Odometry, localization_topic, self._on_odometry, localization_qos
            )

        def _on_odometry(self, msg: Odometry) -> None:
            if not self.accept_samples:
                return
            pose = msg.pose.pose
            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            sample = (float(pose.position.x), float(pose.position.y), yaw)
            if all(math.isfinite(value) for value in sample) and len(self.samples) < 100:
                self.samples.append(sample)

        def wait_for_control_subscribers(self, timeout_s: float) -> tuple[int, int]:
            deadline = time.monotonic() + timeout_s
            counts = (0, 0)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                counts = (
                    self.command_pub.get_subscription_count(),
                    self.pose_pub.get_subscription_count(),
                )
                if counts[0] > 0 and counts[1] > 0:
                    return counts
            return counts

    rclpy.init(args=[])
    node = RelocationNode()
    try:
        counts = node.wait_for_control_subscribers(args.discovery_timeout)
        if counts[0] <= 0 or counts[1] <= 0:
            raise RelocationError(
                "SLAM 重定位控制端未就绪: "
                f"{COMMAND_TOPIC} subscribers={counts[0]}, "
                f"{INITIAL_POSE_TOPIC} subscribers={counts[1]}"
            )
        localization_publishers = node.count_publishers(localization_topic)
        print(
            f"PASS relocation_control command_subscribers={counts[0]} "
            f"pose_subscribers={counts[1]} "
            f"localization_publishers={localization_publishers}",
            flush=True,
        )
        if args.action == "check":
            print("PASS 重定位只读检查完成；未发布任何消息", flush=True)
            return 0

        require_execution_guards(args)
        pixel_x, pixel_y, yaw_deg = validate_initial_pose(
            metadata,
            args.pixel_x,
            args.pixel_y,
            args.yaw_deg,
            allow_non_origin=args.allow_non_origin,
            origin_tolerance_px=args.origin_tolerance_px,
        )
        command = String()
        command.data = f"start_relocalization:{metadata.map_id}"
        node.command_pub.publish(command)
        print(f"PUBLISHED {COMMAND_TOPIC}={command.data}", flush=True)

        delay_deadline = time.monotonic() + args.command_delay
        while rclpy.ok() and time.monotonic() < delay_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        pose = Pose()
        pose.position.x = pixel_x
        pose.position.y = pixel_y
        pose.position.z = 0.0
        pose.orientation.z, pose.orientation.w = yaw_quaternion(yaw_deg)
        node.samples.clear()
        node.accept_samples = True
        node.pose_pub.publish(pose)
        print(
            f"PUBLISHED {INITIAL_POSE_TOPIC} pixel=({pixel_x:g},{pixel_y:g}) "
            f"yaw={yaw_deg:g}deg",
            flush=True,
        )

        deadline = time.monotonic() + args.timeout
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and len(node.samples) < args.samples
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.samples) < args.samples:
            raise RelocationError(
                f"{args.timeout:g}s 内只收到 {len(node.samples)}/{args.samples} 个有效 "
                f"{localization_topic} 样本"
            )
        x, y, yaw = node.samples[-1]
        print(
            f"PASS 重定位收到 {len(node.samples)} 个有效样本: "
            f"x={x:.3f}m y={y:.3f}m yaw={math.degrees(yaw):.1f}deg",
            flush=True,
        )
        print("请人工核对上述米制位姿与机器人真实站位一致后再启动导航", flush=True)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须为正的有限数值")
    return number


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须为正整数")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="X2 真机参数化 SLAM 重定位（check 不发布，execute 受保护）"
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    def add_common(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("--map-id", required=True, help="机器人地图数据库中的地图 ID")
        subparser.add_argument(
            "--map-database", type=Path, default=DEFAULT_MAP_DATABASE
        )
        subparser.add_argument(
            "--discovery-timeout", type=positive_float, default=8.0
        )

    check = subparsers.add_parser("check", help="只读核对地图和 SLAM 控制订阅者")
    add_common(check)

    execute = subparsers.add_parser("execute", help="发布重定位命令并等待定位结果")
    add_common(execute)
    execute.add_argument("--pixel-x", type=float, required=True)
    execute.add_argument("--pixel-y", type=float, required=True)
    execute.add_argument("--yaw-deg", type=float, required=True)
    execute.add_argument(
        "--allow-non-origin",
        action="store_true",
        help="允许使用与建图原点不同、但已现场测量的像素初始位置",
    )
    execute.add_argument(
        "--origin-tolerance-px", type=float, default=1.0
    )
    execute.add_argument("--command-delay", type=positive_float, default=1.0)
    execute.add_argument("--timeout", type=positive_float, default=30.0)
    execute.add_argument("--samples", type=positive_int, default=3)
    execute.add_argument(
        "--confirm-at-pose",
        action="store_true",
        help="确认机器人真实位置和朝向与输入值一致",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        metadata = load_map_metadata(args.map_database, args.map_id)
        print_map(metadata)
        if args.action == "execute":
            # Validate all user-entered values before ROS publishers are created.
            validate_initial_pose(
                metadata,
                args.pixel_x,
                args.pixel_y,
                args.yaw_deg,
                allow_non_origin=args.allow_non_origin,
                origin_tolerance_px=args.origin_tolerance_px,
            )
            require_execution_guards(args)
        return run_ros(args, metadata)
    except (RelocationError, ValueError) as exc:
        print(f"FAIL {exc}", file=sys.stderr, flush=True)
        return 2
    except KeyboardInterrupt:
        print("FAIL 重定位被操作员中断", file=sys.stderr, flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
