#!/usr/bin/env python3
"""One-shot ROS 2 lidar acceptance check used by simulation validation."""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node("raicom_lidar_acceptance")
    result = {}

    def receive(msg):
        offsets = {field.name: field.offset for field in msg.fields}
        points = []
        for index in range(msg.width * msg.height):
            base = index * msg.point_step
            xyz = tuple(struct.unpack_from("<f", msg.data, base + offsets[a])[0]
                        for a in "xyz")
            if all(math.isfinite(v) for v in xyz):
                points.append(xyz)
        result.update(width=msg.width, height=msg.height,
                      frame=msg.header.frame_id, points=points)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, "/aima/sim/lidar/points", receive, qos)
    deadline = time.monotonic() + 5
    while not result and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not result:
        raise SystemExit("FAIL: no lidar message")
    points = result["points"]
    if not points:
        raise SystemExit(f"FAIL: {result['width']} rays but no finite hit")
    distances = [math.sqrt(x*x + y*y + z*z) for x, y, z in points]
    bounds = [(min(p[i] for p in points), max(p[i] for p in points)) for i in range(3)]
    print(f"PASS width={result['width']} height={result['height']} "
          f"frame={result['frame']} finite_hits={len(points)} "
          f"range={min(distances):.3f}..{max(distances):.3f}m "
          f"bounds={bounds} sample={points[:3]}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
