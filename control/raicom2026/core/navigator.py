"""导航模块：TF 定位读取 + 路径规划 + 航向修正移动。

仿真模式：订阅 /aima/hal/odom/state（BEST_EFFORT QoS）
真机模式：用 TF map->base_link + /map_tf_distribution/localization_pose
"""

import math
import struct
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from .locomotion import MotionController
from .mapping import LidarMapper


# ── 比赛场地坐标（参考 raicom_project）─────────────────────
START = (-1.5, -1.5)
START_YAW = math.pi / 2
INTERACT_I = (0.0, 1.70)   # 交互区-I 圆心（官方圆形区域中心 y=1.70）
INTERACT_II = (0.0, 1.00)  # 交互区-II（面向此方向得分，yaw=atan2(-0.7,0)=-90°）
# 中转点：先走到这里再转向，最后倒退泊入 INTERACT_I。
# x=-0.35 预补偿双足原地转的 x 漂移（约 +0.35m）。
STAGING = (-0.35, 1.0)
FINAL_YAW = math.atan2(INTERACT_II[1] - INTERACT_I[1],
                       INTERACT_II[0] - INTERACT_I[0])  # ≈ -90° (面朝南)
WORK_ZONE = (1.0, -1.5)   # 作业区接近点（对齐 raicom_project）


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
        self._odom_origin = None
        # 激光建图在真机才有效；仿真用里程计，不依赖不稳定的激光插件
        if not sim:
            self._mapper = LidarMapper(node, lambda: self._mc.pose)
            self._mc.set_safety_check(self._mapper.safe_to_advance)

        if sim:
            # 仿真用官方里程计（/aima/hal/odom/state），稳定可靠
            node.create_subscription(
                Odometry, "/aima/hal/odom/state", self._on_odom,
                self.ODOM_QOS
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
        if p.z == 0.0 and q.x == q.y == q.z == q.w == 0.0:
            return
        # 里程计直接给出世界坐标，不需要锚点偏移
        self._mc.update_pose(p.x, p.y, p.z, yaw)

    def _on_lidar_pose(self, msg: PointCloud2):
        """Decode sensor world pose reserved in cloud samples zero and one."""
        if msg.width * msg.height < 2 or msg.point_step < 12 or len(msg.data) < 24:
            return
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(axis in offsets for axis in "xyz"):
            return
        points = []
        for index in (0, 1):
            base = index * msg.point_step
            points.append(tuple(struct.unpack_from(
                "<f", msg.data, base + offsets[axis])[0] for axis in "xyz"))
        origin, ahead = points
        if not all(math.isfinite(value) for point in points for value in point):
            return
        dx, dy = ahead[0] - origin[0], ahead[1] - origin[1]
        if math.hypot(dx, dy) < 0.5:
            return
        yaw = math.atan2(dy, dx)
        self._mc.update_pose(origin[0] - 0.10 * math.cos(yaw),
                             origin[1] - 0.10 * math.sin(yaw),
                             origin[2], yaw)

    def _on_pose_stamped(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._mc.update_pose(p.x, p.y, p.z, yaw)

    def reset_map(self):
        if hasattr(self, '_mapper'):
            self._mapper.reset()

    def goto(
        self, target_x: float, target_y: float,
        speed: float = 0.15, timeout: float = 30.0, tolerance: float = 0.15,
    ) -> bool:
        # The CPG's safe translating yaw authority cannot remove a large
        # cross-track error in the final half metre before the north wall.
        # Reach the centre line first, while there is ample turning clearance,
        # then make a slower straight entry into interaction zone I.
        if (self._mc.position is not None
                and abs(target_x - INTERACT_I[0]) < 1e-6
                and abs(target_y - INTERACT_I[1]) < 1e-6
                and self._mc.position[1] < -0.50):
            if not self.goto(
                    *INTERACT_APPROACH, speed=min(speed, 0.35),
                    timeout=min(timeout * 0.65, 150.0), tolerance=0.22):
                return False
            # Final northward approach: laser grid around the interaction zone
            # forces narrow A* corridors that the CPG cannot track smoothly.
            # A straight-line move from the centre approach point is clearer
            # for the gait and still keeps the robot inside the zone.
            self._node.get_logger().info("直走廊接近交互区-I")
            return self._mc.move_toward(
                *INTERACT_I, speed=min(speed, 0.22),
                tolerance=max(tolerance, 0.35), timeout=max(60.0, timeout * 0.35))
        # 仿真模式：直接走直线（里程计定位）
        if self._sim:
            return self._mc.move_toward(
                target_x, target_y, speed=speed, timeout=timeout,
                tolerance=tolerance)
        # 真机模式：激光建图 + A* 规划
        deadline = time.monotonic() + min(2.0, timeout)
        while not self._mapper.ready and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        if self._mc.position and self._mapper.ready:
            route = []
            plan_deadline = time.monotonic() + 2.0
            while not route and time.monotonic() < plan_deadline:
                px, py, _ = self._mc.position
                route = self._mapper.map.plan((px, py), (target_x, target_y))
                if not route:
                    rclpy.spin_once(self._node, timeout_sec=0.1)
            if not route:
                self._node.get_logger().error("激光地图上无可行路径")
                return False
            per_leg = max(30.0, timeout / len(route))
            for index, (x, y) in enumerate(route):
                leg_tolerance = tolerance if index == len(route) - 1 else 0.30
                if not self._mc.move_toward(
                        x, y, speed=speed, timeout=per_leg,
                        tolerance=leg_tolerance):
                    return False
            return True
        self._node.get_logger().warn("激光点云尚不可用，禁止宣称完成自主建图加分项")
        return self._mc.move_toward(
            target_x, target_y, speed=speed, timeout=timeout,
            tolerance=tolerance)

    def goto_waypoints(
        self, waypoints: list, speed: float = 0.15
    ) -> bool:
        for i, (x, y) in enumerate(waypoints):
            self._node.get_logger().info(f"途经点 {i+1}/{len(waypoints)}")
            if not self.goto(x, y, speed=speed):
                return False
        return True

    def task1_enter_zone(self) -> bool:
        """四级进入交互区-I：
        1. 走到中转点 STAGING（粗）
        2. 粗接近 INTERACT_I 前方 0.3m（中）
        3. 原地转到 FINAL_YAW（面朝交互区-II）
        4. dock_at 精确泊入"""
        self._node.get_logger().info("--- 阶段1: 走到中转点 ---")
        if not self._mc.move_toward(*STAGING, speed=0.40, tolerance=0.20, timeout=120.0):
            return False
        # 粗接近：dock_at 是倒退泊入，所以接近点在目标后方（南侧）
        approach = (INTERACT_I[0] + 0.30 * math.cos(FINAL_YAW),
                    INTERACT_I[1] + 0.30 * math.sin(FINAL_YAW))
        self._node.get_logger().info(f"--- 阶段2: 粗接近 {approach} ---")
        if not self._mc.move_toward(*approach, speed=0.25, tolerance=0.15, timeout=60.0):
            return False
        self._node.get_logger().info("--- 阶段3: 原地转向 ---")
        if not self._mc.rotate_to(FINAL_YAW, timeout=20.0):
            return False
        self._node.get_logger().info("--- 阶段4: 倒退泊入 ---")
        return self._mc.dock_at(*INTERACT_I, FINAL_YAW, timeout=60.0)

    def face(self, target_x: float, target_y: float, timeout: float = 100.0) -> bool:
        if self._mc.position is None:
            self._node.get_logger().error("没有定位，无法调整朝向")
            return False
        px, py, _ = self._mc.position
        # 仿真 CPG 步态在小角度旋转时有 ~7° 死区。15° 仍然是人眼可辨的
        # "正面朝向"，且大概率在一次交替踏步周期内收敛。
        return self._mc.rotate_to(
            math.atan2(target_y - py, target_x - px),
            tolerance=math.radians(15), timeout=timeout)

    def face_yaw(self, yaw: float, timeout: float = 100.0) -> bool:
        return self._mc.rotate_to(yaw, timeout=timeout)

    def dock_for_grasp(self, object_xyz, hand: str) -> bool:
        """两级接近桌子：先粗走到作业区，再精确泊入。"""
        object_x, object_y, _ = object_xyz
        dock_x = object_x - 0.25
        dock_y = object_y

        # 第一段：走到桌子前方 0.3m 处
        approach_x = dock_x - 0.30
        approach_y = dock_y
        self._node.get_logger().info(f"粗接近 ({approach_x:.2f}, {approach_y:.2f})")
        if not self._mc.move_toward(approach_x, approach_y, speed=0.15, tolerance=0.15, timeout=90.0):
            return False

        # 第二段：面朝桌子，dock_at 精确泊入
        self._node.get_logger().info("面朝桌子")
        if not self._mc.rotate_to(0.0, timeout=20.0):
            return False
        self._node.get_logger().info(f"精泊入 ({dock_x:.2f}, {dock_y:.2f})")
        return self._mc.dock_at(dock_x, dock_y, 0.0, timeout=60.0)
