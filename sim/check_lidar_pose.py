#!/usr/bin/env python3
"""Print the non-moving world pose carried by the project lidar plugin."""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PoseProbe(Node):
    def __init__(self):
        super().__init__("raicom_lidar_pose_probe")
        self.pose = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, "/aima/sim/lidar/points",
            self._on_cloud, qos,
        )

    def _on_cloud(self, msg):
        if msg.width * msg.height < 2 or msg.point_step < 12:
            return
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(axis in offsets for axis in "xyz"):
            return
        points = []
        endian = ">f" if msg.is_bigendian else "<f"
        for index in (0, 1):
            base = index * msg.point_step
            points.append(tuple(struct.unpack_from(
                endian, msg.data, base + offsets[axis])[0] for axis in "xyz"))
        origin, ahead = points
        if not all(math.isfinite(v) for point in points for v in point):
            return
        yaw = math.atan2(ahead[1] - origin[1], ahead[0] - origin[0])
        self.pose = (
            origin[0] - 0.10 * math.cos(yaw),
            origin[1] - 0.10 * math.sin(yaw),
            origin[2], yaw,
        )


def main():
    rclpy.init()
    node = PoseProbe()
    deadline = time.monotonic() + 5.0
    while node.pose is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        print("lidar_pose=unavailable")
        result = 1
    else:
        x, y, z, yaw = node.pose
        print(f"lidar_pose=({x:.4f}, {y:.4f}, {z:.4f}, {yaw:.4f})")
        result = 0
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(result)


if __name__ == "__main__":
    main()
