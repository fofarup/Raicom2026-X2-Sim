"""速度控制 + 输入源注册 + 航向修正。"""

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class InputSource:
    def __init__(self, node: Node, name: str = "raicom2026", priority: int = 60):
        self._node = node
        self._client = node.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self._name = name
        self._priority = priority

    def register(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=10.0):
            return False
        req = SetMcInputSource.Request()
        import rclpy
        # ADD. If a previous process left the same name behind, DELETE it and
        # ADD again: this MC version acknowledges MODIFY but keeps old values.
        registered = False
        for action in (1001, 1003, 1001):
            req.action.value = action
            req.input_source.name = self._name
            req.input_source.priority = self._priority
            # Official examples use 1000 ms.  A longer lease lets the last
            # non-zero command survive process exit and move a freshly reset
            # simulator before the next controller starts.
            req.input_source.timeout = 1000
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
            response = future.result() if future.done() else None
            state = response.response.state.value if response is not None else 0
            code = response.response.header.code if response is not None else -1
            self._node.get_logger().info(
                f"输入源 action={action} priority={self._priority} "
                f"state={state} code={code}")
            # 当前 MC 服务成功时 header.code=0，但部分版本没有填写 state。
            if action == 1003:
                continue
            if action == 1001:
                registered = (response is not None and code == 0 and
                              state in (0, 1))
        return registered


class MotionController:
    TOPIC = "/aima/mc/locomotion/velocity"
    # Reset-isolated calibration: the CPG's measured displacement direction is
    # consistently about 8--18 degrees counter-clockwise from the reported
    # pelvis/camera yaw. Use the conservative centre of that interval until a
    # segment has travelled far enough to measure its actual course.
    COURSE_YAW_OFFSET = math.radians(12.0)

    def __init__(self, node: Node, source_name: str = "raicom2026"):
        self._node = node
        self._source = source_name
        self._pub = node.create_publisher(McLocomotionVelocity, self.TOPIC, 10)
        self._position = None  # (x, y, z, yaw)
        self._pose_received_at = None
        self._safety_check = None

    def set_safety_check(self, check):
        self._safety_check = check

    def update_pose(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        self._position = (x, y, z, yaw)
        self._pose_received_at = time.monotonic()

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        if self._position is None:
            return None
        return (self._position[0], self._position[1], self._position[2])

    @property
    def pose(self) -> Optional[Tuple[float, float, float, float]]:
        """Latest complete map-frame pose (x, y, z, yaw)."""
        return self._position

    @property
    def yaw(self) -> Optional[float]:
        if self._position is None:
            return None
        return self._position[3]

    def publish(self, forward: float, angular: float = 0.0,
                lateral: float = 0.0):
        msg = McLocomotionVelocity()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.source = self._source
        msg.forward_velocity = forward
        msg.lateral_velocity = lateral
        msg.angular_velocity = angular
        self._pub.publish(msg)

    def pose_is_fresh(self, max_age: float = 0.5) -> bool:
        return (self._pose_received_at is not None and
                time.monotonic() - self._pose_received_at <= max_age)

    def stop(self, duration: float = 1.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish(0.0)
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(0.02)

    # ── 参考 raicom_project 导航参数 ──
    YAW_TOLERANCE = math.radians(4.0)    # 朝向容差
    POS_TOLERANCE = 0.12                 # 位置容差
    DOCK_TOLERANCE = 0.08                # 泊车位置容差
    DOCK_OVERSHOOT = 0.20                # 泊车过冲量 (m)
    HEADING_GATE = math.radians(25.0)    # 超过此角度先原地转
    POSE_MAX_AGE = 0.6                   # 定位数据最大有效期

    def move_toward(
        self, target_x: float, target_y: float,
        speed: float = 0.30, tolerance: float = 0.12,
        timeout: float = 30.0, obstacle_check: bool = True,
    ) -> bool:
        """参考 raicom_project: drive_to。
        朝向偏差 >25° 先原地转，否则边走边修。"""
        self._node.get_logger().info(f"移动至 ({target_x:.2f}, {target_y:.2f})")
        deadline = time.monotonic() + timeout
        import rclpy
        cnt = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            cnt += 1
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.01)
                continue
            px, py, pz = self.position
            if abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error("边界急停")
                self.stop(0.5); return False
            if pz < 0.45:
                self._node.get_logger().error("跌倒急停")
                self.stop(0.5); return False
            dx, dy = target_x - px, target_y - py
            dist = math.hypot(dx, dy)
            if dist < tolerance:
                self._node.get_logger().info("已到达")
                self.stop(1.0); return True
            target_yaw = math.atan2(dy, dx)
            current_yaw = self.yaw if self.yaw is not None else target_yaw
            heading_err = math.atan2(math.sin(target_yaw - current_yaw),
                                     math.cos(target_yaw - current_yaw))
            # 朝向偏差大 → 只转不走
            if abs(heading_err) > self.HEADING_GATE:
                fwd = 0.0
            else:
                fwd = max(0.06, min(speed, 0.55 * dist))
                fwd *= max(0.25, math.cos(heading_err))
            angular = max(-0.40, min(0.40, 1.4 * heading_err))
            if cnt % 15 == 0:
                self._node.get_logger().info(
                    f"  pos=({px:.2f},{py:.2f}) dist={dist:.2f} "
                    f"head_err={math.degrees(heading_err):.0f}deg "
                    f"fwd={fwd:.2f} ang={angular:+.2f}")
            self.publish(fwd, angular)
            time.sleep(0.02)
        self._node.get_logger().warn("移动超时！")
        self.stop(1.0)
        return False

    def dock_at(self, target_x: float, target_y: float,
                target_yaw: float, timeout: float = 30.0) -> bool:
        """参考 raicom_project: dock_at。低速倒退泊入目标点，同时保持朝向。
        控制点放在目标后方 0.20m，让机器人自然倒退进目标。"""
        self._node.get_logger().info(f"泊入 ({target_x:.2f}, {target_y:.2f}) yaw={math.degrees(target_yaw):.0f}deg")
        deadline = time.monotonic() + timeout
        import rclpy
        cnt = 0
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            cnt += 1
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.01); continue
            px, py, pz = self.position
            if pz < 0.45:
                self.stop(0.5); return False
            # 控制点 = 目标点后方 0.20m（机器人倒退进去）
            ctrl_x = target_x - math.cos(target_yaw) * self.DOCK_OVERSHOOT
            ctrl_y = target_y - math.sin(target_yaw) * self.DOCK_OVERSHOOT
            dx, dy = ctrl_x - px, ctrl_y - py
            dist = math.hypot(dx, dy)
            yaw_err = math.atan2(math.sin(target_yaw - self.yaw),
                                 math.cos(target_yaw - self.yaw))
            # 检查停稳
            ok_dist = dist < self.DOCK_TOLERANCE
            ok_yaw = abs(yaw_err) <= self.YAW_TOLERANCE
            if ok_dist and ok_yaw:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.5:
                    self._node.get_logger().info("泊入完成")
                    self.stop(1.0)
                    return True
            else:
                stable_since = None
            # 走倒退方向
            body_forward = (math.cos(self.yaw) * dx + math.sin(self.yaw) * dy) / max(dist, 0.01)
            fwd = max(-0.20, min(0.20, 0.75 * body_forward))
            angular = max(-0.25, min(0.25, 1.0 * yaw_err))
            if cnt % 15 == 0:
                self._node.get_logger().info(
                    f"  dist={dist:.2f} yaw_err={math.degrees(yaw_err):.0f}deg fwd={fwd:.2f}")
            self.publish(fwd, angular)
            time.sleep(0.02)
        self._node.get_logger().warn("泊入超时！")
        self.stop(1.0)
        return False

    def rotate_to(self, target_yaw: float, tolerance: float = math.radians(7),
                  timeout: float = 12.0) -> bool:
        """原地转向：前后=0，侧向交替踏步。最低角速度 0.10 突破 CPG 死区。"""
        import rclpy
        anchor = None if self.position is None else self.position[:2]
        count = 0
        started = time.monotonic()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            count += 1
            if self.yaw is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.02)
                continue
            if self.position is not None and self.position[2] < 0.45:
                self._node.get_logger().error("转向跌倒急停")
                self.stop(0.5)
                return False
            px, py, _ = self.position
            if abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error("转向边界急停")
                self.stop(0.5)
                return False
            error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))
            if abs(error) <= tolerance:
                self.stop(0.8)
                return True
            # 最低 0.10 rad/s 突破固件死区 (~0.03 rad/s)，上限 0.50
            sign = 1.0 if error > 0 else -1.0
            magnitude = max(0.10, abs(1.0 * error))
            angular = sign * min(0.50, magnitude)
            # 侧向踏步 0.3s 交替
            phase = int((time.monotonic() - started) / 0.30)
            lateral = 0.06 if phase % 2 == 0 else -0.06
            if anchor is None:
                anchor = (px, py)
            drift = math.hypot(anchor[0] - px, anchor[1] - py)
            if count % 25 == 0:
                self._node.get_logger().info(
                    f"  rotate_err={math.degrees(error):.1f}deg "
                    f"drift={drift:.2f}m angular={angular:+.2f}")
            self.publish(0.0, angular, lateral)
            time.sleep(0.02)
        self.stop(1.0)
        self._node.get_logger().warn("转向超时")
        return False

    def move_axis(self, axis: str, target: float, speed: float = 0.20,
                  tolerance: float = 0.06, timeout: float = 45.0) -> bool:
        """Walk a monotonic world-X/Y segment after aligning its heading.

        This is used only for the obstacle-free table docking corridor.  It
        avoids the ill-conditioned 2-D point controller near a target, where
        the biped's unavoidable translation during turns otherwise causes an
        orbit.
        """
        if axis not in ("x", "y"):
            raise ValueError(axis)
        import rclpy
        if self.position is None:
            return False
        index = 0 if axis == "x" else 1
        current = self.position[index]
        if abs(target - current) <= tolerance:
            return True
        direction = 1.0 if target > current else -1.0
        if axis == "x":
            heading = 0.0 if target > current else math.pi
        else:
            heading = math.pi / 2 if target > current else -math.pi / 2
        if not self.rotate_to(
                heading, tolerance=math.radians(16), timeout=30.0):
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.02)
                continue
            px, py, pz = self.position
            if pz < 0.45 or abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error("轴向移动触发姿态/边界急停")
                self.stop(0.5)
                return False
            error = target - self.position[index]
            # Stop on entry into the tolerance band *or* immediately after a
            # crossing. Continuing with positive forward velocity after an
            # overshoot would carry the robot farther away forever.
            if direction * error <= tolerance:
                self.stop(0.8)
                return True
            yaw_error = math.atan2(math.sin(heading - (self.yaw or heading)),
                                   math.cos(heading - (self.yaw or heading)))
            angular = max(-0.052, min(0.052, yaw_error * 0.08))
            self.publish(max(0.20, speed), angular, 0.0)
            time.sleep(0.02)
        self.stop(0.8)
        self._node.get_logger().warn(f"{axis} 轴移动超时")
        return False

    def strafe_world_y(self, target_y: float, tolerance: float = 0.08,
                       timeout: float = 25.0) -> bool:
        """Correct table-lane cross-track error while holding world yaw zero.

        The final table approach is the one place where the CPG's weak lateral
        channel is useful: it changes world Y without another 90-degree turn.
        Position and yaw are both closed from the live lidar world pose.
        """
        import rclpy
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.02)
                continue
            px, py, pz = self.position
            if pz < 0.45 or abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error("横向停靠触发姿态/边界急停")
                self.stop(0.5)
                return False
            error = target_y - py
            if abs(error) <= tolerance:
                self.stop(0.8)
                return True
            yaw_error = math.atan2(math.sin(-(self.yaw or 0.0)),
                                   math.cos(-(self.yaw or 0.0)))
            # If lateral coupling has accumulated too much yaw, settle and
            # realign before continuing rather than rotating while translating.
            if abs(yaw_error) > math.radians(20):
                self.stop(0.4)
                if not self.rotate_to(
                        0.0, tolerance=math.radians(16), timeout=15.0):
                    return False
                continue
            lateral = math.copysign(0.50, error)
            angular = max(-0.052, min(0.052, yaw_error * 0.10))
            self.publish(0.0, angular, lateral)
            time.sleep(0.02)
        self.stop(0.8)
        self._node.get_logger().warn("横向停靠超时")
        return False
