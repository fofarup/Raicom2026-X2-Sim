#!/usr/bin/env python3
"""One-shot runtime acceptance check for blank-map lidar mapping and A*."""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from core.mapping import LidarMapper
from core.navigator import INTERACT_I


def main():
    rclpy.init()
    node = Node("raicom_mapping_acceptance")
    pose = [None]

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                         1 - 2 * (q.y*q.y + q.z*q.z))
        pose[0] = (p.x, p.y, p.z, yaw)

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(Odometry, "/aima/hal/odom/state", odom, qos)
    mapper = LidarMapper(node, lambda: pose[0])
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not mapper.ready or pose[0] is None:
        raise SystemExit("FAIL: map or odometry unavailable")
    occupied = int(np.count_nonzero(mapper.map.grid == 100))
    free = int(np.count_nonzero(mapper.map.grid == 0))
    route = mapper.map.plan(pose[0][:2], INTERACT_I)
    occupied_cells = np.argwhere(mapper.map.grid == 100)
    occupied_ranges = [math.hypot(mapper.map.world((int(x), int(y)))[0] - pose[0][0],
                                  mapper.map.world((int(x), int(y)))[1] - pose[0][1])
                       for y, x in occupied_cells]
    nearest = min(occupied_ranges, default=math.inf)
    if occupied < 10 or free < 100 or not route:
        start = mapper.map.cell(*pose[0][:2])
        blocked = mapper.map.inflated()
        sx, sy = start
        for y in range(sy + 12, sy - 13, -1):
            print("".join("S" if (x, y) == start else
                          "#" if 0 <= y < mapper.map.height and
                                 0 <= x < mapper.map.width and blocked[y, x] else "."
                          for x in range(sx - 12, sx + 13)))
        raise SystemExit(
            f"FAIL: occupied={occupied} free={free} route={len(route)} "
            f"nearest_occupied={nearest:.3f}")
    print(f"PASS occupied={occupied} free={free} route_waypoints={len(route)} "
          f"front_clearance={mapper.front_clearance:.3f}m")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
