#!/usr/bin/env python3
"""Print reserved physical object and claw-centre coordinates."""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node("raicom_physical_geometry")
    result = None

    def receive(msg):
        nonlocal result
        if msg.width * msg.height < 7:
            return
        offsets = {field.name: field.offset for field in msg.fields}
        endian = ">f" if msg.is_bigendian else "<f"
        result = []
        for index in range(2, 7):
            base = index * msg.point_step
            result.append(tuple(struct.unpack_from(
                endian, msg.data, base + offsets[axis])[0] for axis in "xyz"))

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, "/aima/sim/lidar/points", receive, qos)
    deadline = time.monotonic() + 3.0
    while result is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    names = ("medicine", "mug", "bread", "left_claw", "right_claw")
    if result is None or not all(math.isfinite(v) for p in result for v in p):
        raise SystemExit("physical_geometry=unavailable")
    for name, point in zip(names, result):
        print(f"{name}=({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f})")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
