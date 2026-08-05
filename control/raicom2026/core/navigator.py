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


# ── 比赛场地坐标 ──────────────────────────────────────────────
START = (-1.5, -1.5)
START_YAW = math.pi / 2
# 圆形交互区中心为 y=1.70；中心点距后墙内沿仅约 0.15 m。取区内
# y=1.55 的安全点，既保持机器人整体进入标识区，又给身体留出墙距。
INTERACT_I = (0.0, 1.55)
INTERACT_II = (0.0, 1.00)
INTERACT_APPROACH = (0.0, 0.55)
# 停在桌前而不是桌子中心；机器人面向 +x 方向取物。
WORK_ZONE = (0.65, -0.85)


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
        """Make a short, precision-controlled approach to the table front."""
        object_x, object_y, _ = object_xyz
        lateral = 0.30 if hand == "left" else -0.30
        # The table front is aligned with the map y-axis and must be approached
        # facing +x. Computing this from the *current* yaw makes the target move
        # after the gait turns and produces an orbit in front of the table.
        # IK sweep with the physical 13 cm tool point gives a hard low-height
        # reach limit near 0.32 m. A 0.22 m pelvis-to-object standoff remains
        # clear of the measured table front while keeping the contact target
        # comfortably inside that workspace, including up to 10 degrees yaw.
        dock_x = object_x - 0.22
        dock_y = object_y - lateral
        self._node.get_logger().info(
            f"精确停靠 {hand}: ({dock_x:.2f}, {dock_y:.2f})")
        # Keep the final manoeuvre out of the general 2-D point controller:
        # below roughly 50 cm, its course estimate becomes ill-conditioned and
        # can orbit the target. Align once, use the weak lateral channel only
        # for the table-lane Y correction, then make a monotonic +X approach.
        if not self._mc.rotate_to(
                0.0, tolerance=math.radians(16), timeout=30.0):
            return False
        # The table is intentionally closer than normal navigation clearance.
        # Stop from signed world-X progress; live post-US pose is still used by
        # arm IK for the remaining centimetres.
        if not self._mc.move_axis(
                "x", dock_x, speed=0.20, tolerance=0.025, timeout=25.0):
            return False
        # Forward gait can couple several centimetres into Y, so cross-track
        # correction must be the last translation, not the first.
        if not self._mc.strafe_world_y(
                dock_y, tolerance=0.06, timeout=45.0):
            return False
        if not self._mc.rotate_to(
                0.0, tolerance=math.radians(16), timeout=20.0):
            return False
        return True
