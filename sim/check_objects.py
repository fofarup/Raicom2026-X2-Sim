#!/usr/bin/env python3
"""Inspect table-object clusters from the live world-frame lidar cloud."""
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def clusters(points, radius=0.09):
    pending = set(range(len(points)))
    result = []
    while pending:
        group, frontier = [], [pending.pop()]
        while frontier:
            index = frontier.pop()
            group.append(points[index])
            nearby = [other for other in pending
                      if math.dist(points[index], points[other]) <= radius]
            for other in nearby:
                pending.remove(other)
                frontier.append(other)
        result.append(group)
    return sorted(result, key=len, reverse=True)


def main():
    rclpy.init()
    node = Node("raicom_object_cloud_check")
    result = []

    def receive(msg):
        offsets = {field.name: field.offset for field in msg.fields}
        for index in range(7, msg.width * msg.height):
            base = index * msg.point_step
            point = tuple(struct.unpack_from("<f", msg.data, base + offsets[a])[0]
                          for a in "xyz")
            x, y, z = point
            if (all(math.isfinite(v) for v in point) and
                    1.15 <= x <= 1.85 and -1.90 <= y <= -0.90 and
                    0.56 <= z <= 0.85):
                result.append(point)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(PointCloud2, "/aima/sim/lidar/points", receive, qos)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if result:
            break
    groups = clusters(result)
    print(f"candidate_points={len(result)} clusters={len(groups)}")
    for group in groups[:10]:
        lower = [min(point[i] for point in group) for i in range(3)]
        upper = [max(point[i] for point in group) for i in range(3)]
        center = [sum(point[i] for point in group) / len(group) for i in range(3)]
        print(f"n={len(group)} center={[round(v,3) for v in center]} "
              f"size={[round(upper[i]-lower[i],3) for i in range(3)]}")
    if not result:
        raise SystemExit("FAIL: no visible points above table")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
