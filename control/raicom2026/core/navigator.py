"""导航模块：TF 定位读取 + 路径规划 + 航向修正移动。

仿真模式：订阅 /aima/hal/odom/state（BEST_EFFORT QoS）
真机模式：用 TF map->base_link + /map_tf_distribution/localization_pose
"""

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from .locomotion import MotionController


# ── 比赛场地坐标 ──────────────────────────────────────────────
START = (-1.5, -1.5)
INTERACT_I = (0.0, 1.0)
INTERACT_II = (0.5, 1.0)
WORK_ZONE = (1.5, -1.4)


class Navigator:
    """TF/里程计定位 + 移动控制。"""

    ODOM_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def __init__(self, node: Node, mc: MotionController, sim: bool = True):
        self._node = node
        self._mc = mc
        self._sim = sim

        if sim:
            # 仿真用 odometry
            node.create_subscription(
                Odometry, "/aima/hal/odom/state", self._on_odom, self.ODOM_QOS
            )
        else:
            # 真机用 TF 定位位姿
            node.create_subscription(
                PoseStamped,
                "/map_tf_distribution/localization_pose",
                self._on_pose_stamped,
                10,
            )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._mc.update_pose(p.x, p.y, p.z, yaw)

    def _on_pose_stamped(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._mc.update_pose(p.x, p.y, p.z, yaw)

    def goto(
        self, target_x: float, target_y: float,
        speed: float = 0.15, timeout: float = 30.0,
    ) -> bool:
        return self._mc.move_toward(target_x, target_y, speed=speed, timeout=timeout)

    def goto_waypoints(
        self, waypoints: list, speed: float = 0.15
    ) -> bool:
        for i, (x, y) in enumerate(waypoints):
            self._node.get_logger().info(f"途经点 {i+1}/{len(waypoints)}")
            if not self.goto(x, y, speed=speed):
                return False
        return True
