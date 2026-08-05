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

    def move_toward(
        self, target_x: float, target_y: float,
        speed: float = 0.35, tolerance: float = 0.15,
        timeout: float = 30.0, obstacle_check: bool = True,
    ) -> bool:
        self._node.get_logger().info(f"移动至 ({target_x:.2f}, {target_y:.2f})")
        deadline = time.monotonic() + timeout
        import rclpy
        cnt = 0
        prealigned = False
        start_escaped = False
        course_heading = None
        course_anchor = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            cnt += 1
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.01)
                continue
            if (obstacle_check and self._safety_check is not None and
                    not self._safety_check()):
                self._node.get_logger().warn("激光检测到前方障碍，停止并请求重新规划")
                self.stop(0.5)
                return False
            px, py, _ = self.position
            if abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error(
                    f"接近场地边界 ({px:.2f}, {py:.2f})，急停")
                self.stop(0.5)
                return False
            if self.position[2] < 0.45:
                self._node.get_logger().error("检测到骨盆高度过低，判定跌倒并急停")
                self.stop(0.5)
                return False
            if (not start_escaped and px < -1.30 and py < -1.20):
                if not self._escape_start_corner(deadline):
                    return False
                start_escaped = True
                prealigned = True
                course_anchor = None
                course_heading = None
                continue
            dist = math.hypot(target_x - px, target_y - py)
            if dist < tolerance:
                self._node.get_logger().info("已到达")
                self.stop(1.0)
                return True
            target_yaw = math.atan2(target_y - py, target_x - px)
            if course_anchor is None:
                course_anchor = (px, py)
                body_yaw = self.yaw if self.yaw is not None else target_yaw
                course_heading = body_yaw + self.COURSE_YAW_OFFSET
            course_dx, course_dy = px - course_anchor[0], py - course_anchor[1]
            if math.hypot(course_dx, course_dy) >= 0.08:
                measured = math.atan2(course_dy, course_dx)
                # Circular low-pass filtering rejects individual 5 Hz lidar
                # pose jumps without hiding sustained cross-track drift.
                delta = math.atan2(math.sin(measured - course_heading),
                                   math.cos(measured - course_heading))
                course_heading += 0.65 * delta
                course_anchor = (px, py)
            yaw_err = target_yaw - course_heading
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            # Re-align whenever meaningful translation remains.  A fixed
            # 0.50 m gate let precision moves orbit their target with 90--170
            # degree heading error instead of turning to face it.
            if dist > max(0.75, tolerance + 0.20) and abs(yaw_err) > math.radians(45):
                # Leave a wall-side start pose along the already stable body
                # heading before asking the gait for a large yaw change.
                yaw_now = self.yaw or 0.0
                inward = (math.cos(yaw_now) * -px + math.sin(yaw_now) * -py)
                if (not prealigned and max(abs(px), abs(py)) > 1.20 and
                        inward > 0.0):
                    if px < -1.30 and py < -1.20:
                        # Official start is in the south-west corner. A
                        # short 60-degree heading followed by a north-east leg
                        # creates clearance from both walls before the large
                        # clockwise turn.
                        if not self.rotate_to(
                                math.radians(60),
                                tolerance=math.radians(12), timeout=15.0):
                            return False
                        rclpy.spin_once(self._node, timeout_sec=0.05)
                        px, py, _ = self.position
                        stage_x, stage_y = px + 0.35, py + 0.55
                    else:
                        stage_x = px + 0.65 * math.cos(yaw_now)
                        stage_y = py + 0.65 * math.sin(yaw_now)
                    remaining = deadline - time.monotonic()
                    if not self.move_toward(
                            stage_x, stage_y, speed=0.35, tolerance=0.30,
                            timeout=min(30.0, remaining - 1.0),
                            obstacle_check=obstacle_check):
                        return False
                remaining = deadline - time.monotonic()
                if remaining <= 5.0 or not self.rotate_to(
                        target_yaw - self.COURSE_YAW_OFFSET,
                        tolerance=math.radians(25),
                        timeout=min(40.0, remaining - 1.0)):
                    return False
                prealigned = True
                rclpy.spin_once(self._node, timeout_sec=0.05)
                px, py, _ = self.position
                course_anchor = (px, py)
                body_yaw = self.yaw if self.yaw is not None else target_yaw
                course_heading = body_yaw + self.COURSE_YAW_OFFSET
                continue
            prealigned = True
            # At normal forward speed this CPG largely suppresses yaw (a
            # reset-isolated +0.15 command changed only 3.9 degrees in 5 s).
            # For a large measured course error use the separately calibrated
            # stable turning gait: 0.10 m/s forward with <=0.30 rad/s yaw.
            turning_course = abs(yaw_err) > math.radians(30)
            if turning_course:
                angular = max(-0.30, min(0.30, yaw_err * 0.35))
            else:
                angular = max(-0.052, min(0.052, yaw_err * 0.12))
            yaw = self.yaw or 0.0
            # The bundled CPG is not holonomic despite exposing three velocity
            # fields. A reset-isolated calibration measured pure forward motion
            # at 0.885 m / 5 s with under 1 mm cross-track error, while lateral
            # coupling caused target-side orbits. Turn first, then walk forward.
            gait = (0.10 if turning_course else
                    max(0.20, speed * min(1.0, dist / 0.60)))
            fwd = gait
            lateral = 0.0
            if cnt % 15 == 0:
                self._node.get_logger().info(
                    f"  pose=({px:.2f},{py:.2f},{math.degrees(yaw):.0f}°) "
                    f"course={math.degrees(course_heading):.0f}° "
                    f"dist={dist:.2f} course_err={math.degrees(yaw_err):.0f}° "
                    f"cmd=({fwd:+.2f},{lateral:+.2f},{angular:+.3f})")
            self.publish(fwd, angular, lateral)
            time.sleep(0.02)
        self._node.get_logger().warn("移动超时！")
        self.stop(1.0)
        return False

    def _escape_start_corner(self, outer_deadline: float) -> bool:
        """Leave the south-west start using the reset-calibrated straight gait.

        Turning in the corner translates unpredictably and can touch the west
        wall. Reset-isolated calibration of forward=0.30/lateral=-0.50 moved
        0.98 m north and 0.42 m east in five seconds while remaining upright.
        """
        import rclpy
        deadline = min(outer_deadline - 2.0, time.monotonic() + 12.0)
        self._node.get_logger().info("官方起点直行脱离墙角")
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            if self.position is None or not self.pose_is_fresh():
                self.publish(0.0)
                time.sleep(0.02)
                continue
            px, py, pz = self.position
            if pz < 0.45 or px < -1.74 or py < -1.74:
                self._node.get_logger().error(
                    f"起点脱离触发安全停止 ({px:.2f}, {py:.2f}, z={pz:.2f})")
                self.stop(0.5)
                return False
            if py >= -0.88:
                self.stop(0.8)
                return True
            self.publish(0.30, 0.0, -0.50)
            time.sleep(0.02)
        self.stop(0.5)
        self._node.get_logger().error("起点直行脱离超时")
        return False

    def rotate_to(self, target_yaw: float, tolerance: float = math.radians(5),
                  timeout: float = 12.0) -> bool:
        """小幅交替踏步转到绝对航向，并确认停止。

        官方 RL 步态对纯角速度有死区，必须同时给出很小的平移速度。
        转向会产生少量平移；到达航向后直接交还给随后地图坐标导航
        消除该误差。这里若另起一次“回转向起点”，会在旧速度尚未完全
        衰减时反向追逐一个近点，实测容易过冲并破坏步态稳定性。
        """
        import rclpy
        anchor = None if self.position is None else self.position[:2]
        count = 0
        initial_forward_sign = None
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
                self._node.get_logger().error("转向时检测到跌倒，立即停止")
                self.stop(0.5)
                return False
            px, py, _ = self.position
            if abs(px) > 1.78 or abs(py) > 1.78:
                self._node.get_logger().error(
                    f"转向接近场地边界 ({px:.2f}, {py:.2f})，急停")
                self.stop(0.5)
                return False
            error = math.atan2(math.sin(target_yaw - self.yaw),
                               math.cos(target_yaw - self.yaw))
            if abs(error) <= tolerance:
                self.stop(0.8)
                return True
            angular = max(-0.30, min(0.30, 0.30 * error))
            if anchor is None:
                anchor = (px, py)
            drift = math.hypot(anchor[0] - px, anchor[1] - py)
            # Pure angular velocity falls inside the bundled CPG's dead band,
            # but holding one forward sign for a 90-degree turn drifts roughly
            # half a metre. Reset-isolated calibration showed that alternating
            # +0.10/-0.10 every 0.40 s preserves yaw authority while reducing
            # 90-degree translational drift to 6.2 cm.
            heading_dot_centre = (math.cos(self.yaw) * -px
                                  + math.sin(self.yaw) * -py)
            nominal_sign = 1.0 if heading_dot_centre >= 0.0 else -1.0
            if initial_forward_sign is None:
                initial_forward_sign = nominal_sign
            phase = int((time.monotonic() - started) / 0.40)
            alternating_sign = 1.0 if phase % 2 == 0 else -1.0
            forward = 0.10 * initial_forward_sign * alternating_sign
            lateral, turn_command = 0.0, angular
            if count % 25 == 0:
                self._node.get_logger().info(
                    f"  rotate_err={math.degrees(error):.0f}° "
                    f"drift={drift:.2f} radius={math.hypot(px, py):.2f} "
                    f"forward={forward:+.2f}")
            self.publish(forward, turn_command, lateral)
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
