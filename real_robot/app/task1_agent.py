#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RAICOM 2026 正式任务 1：自主导航并在交互区 I 正确就位。"""

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aimdk_msgs.msg import McActionCommand, McLocomotionVelocity, MessageHeader, RequestHeader
from aimdk_msgs.srv import GetCurrentInputSource, SetMcAction, SetMcInputSource
from robot_profile import load_robot_profile


ROBOT_PROFILE = load_robot_profile()
NAVIGATION_PROFILE = ROBOT_PROFILE["navigation"]
ZONES = {name: tuple(xy) for name, xy in NAVIGATION_PROFILE["zones"].items()}

# 真机转向漂移和横移能力必须现场低速标定，禁止沿用仿真数值。
TURN_DRIFT_PRECOMPENSATION_X = float(
    NAVIGATION_PROFILE["task1_turn_drift_precompensation_x_m"]
)
PRETURN_POINT = NAVIGATION_PROFILE.get("task1_preturn_xy")
INTERACTION_DOCK_OFFSET_Y_M = float(
    NAVIGATION_PROFILE["task1_interaction_dock_offset_y_m"]
)
INTERACTION_DOCK_OFFSET_X_M = float(
    NAVIGATION_PROFILE.get("task1_interaction_dock_offset_x_m", 0.0)
)
SUPPORTS_LATERAL = bool(NAVIGATION_PROFILE["supports_lateral_velocity"])
MAX_FORWARD = float(NAVIGATION_PROFILE["max_forward_velocity_mps"])
MAX_LATERAL = float(NAVIGATION_PROFILE["max_lateral_velocity_mps"])
MAX_ANGULAR = float(NAVIGATION_PROFILE["max_angular_velocity_rps"])


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Task1Agent(Node):
    POSE_TOPIC = ROBOT_PROFILE["topics"]["localization_pose"]
    ODOM_TOPIC = ROBOT_PROFILE["topics"]["odometry"]
    POSITION_TOLERANCE_M = 0.15
    DOCK_POSITION_TOLERANCE_M = 0.15
    DOCK_CONTROL_OVERSHOOT_M = 0.20
    YAW_TOLERANCE_RAD = math.radians(4.0)
    POSE_MAX_AGE_S = 0.6

    def __init__(self, node_name: str = "task1_agent"):
        super().__init__(node_name)
        self.get_logger().info(
            f"机器人配置: {ROBOT_PROFILE['profile_name']} ({ROBOT_PROFILE['_path']})"
        )
        self._pose = None
        self._pose_received_at = 0.0
        self._pose_source = ""
        self._pose_priority = 0
        self._pose_lock = threading.Lock()
        self._has_source = False

        self._vel_pub = self.create_publisher(
            McLocomotionVelocity, ROBOT_PROFILE["topics"]["locomotion_velocity"], 10
        )
        localization_type = ROBOT_PROFILE["topics"]["localization_pose_type"]
        if localization_type == "geometry_msgs/msg/PoseStamped":
            self.create_subscription(PoseStamped, self.POSE_TOPIC, self._on_pose, 10)
        elif localization_type == "nav_msgs/msg/Odometry":
            self.create_subscription(
                Odometry, self.POSE_TOPIC, self._on_localization_odom, qos_profile_sensor_data
            )
        else:
            raise RuntimeError(f"不支持的定位消息类型: {localization_type}")
        self.create_subscription(
            Odometry, self.ODOM_TOPIC, self._on_odom, qos_profile_sensor_data
        )
        self._mc = self.create_client(SetMcAction, ROBOT_PROFILE["services"]["mc_action"])
        self._src = self.create_client(
            SetMcInputSource, ROBOT_PROFILE["services"]["mc_input_source"]
        )
        self._current_src = self.create_client(
            GetCurrentInputSource, "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
        )

    def _on_pose(self, msg: PoseStamped):
        self._store_pose(msg.pose, "map localization", priority=2)

    def _on_odom(self, msg: Odometry):
        self._store_pose(msg.pose.pose, "AimDK odometry", priority=1)

    def _on_localization_odom(self, msg: Odometry):
        self._store_pose(msg.pose.pose, "SLAM localization", priority=2)

    def _store_pose(self, pose, source: str, priority: int):
        now = time.monotonic()
        # 新鲜的 map 定位优先于里程计；map 定位失效后自动回退里程计。
        with self._pose_lock:
            if priority < self._pose_priority and now - self._pose_received_at <= self.POSE_MAX_AGE_S:
                return
        q = pose.orientation
        sample = (
            float(pose.position.x),
            float(pose.position.y),
            quaternion_yaw(q.x, q.y, q.z, q.w),
        )
        with self._pose_lock:
            self._pose = sample
            self._pose_received_at = now
            self._pose_source = source
            self._pose_priority = priority

    def _current_pose(self):
        with self._pose_lock:
            return self._pose, time.monotonic() - self._pose_received_at, self._pose_source

    def wait_for_pose(self, timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose, age, source = self._current_pose()
            # Global Task1 targets are expressed in the SLAM map frame.  Leg
            # odometry is useful only as diagnostics and must never start a
            # global navigation run before the configured SLAM pose arrives.
            if pose is not None and age <= self.POSE_MAX_AGE_S and self._pose_priority >= 2:
                self.get_logger().info(
                    f"定位就绪[{source}]: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f}°"
                )
                return True
        self.get_logger().error(
            f"{timeout_s:.0f}s 内未收到定位: {self.POSE_TOPIC} 或 {self.ODOM_TOPIC}"
        )
        return False

    def register_source(self) -> bool:
        if self._has_source:
            return True
        if not self._src.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMcInputSource 服务不可用")
            return False

        def request(action: int, label: str) -> bool:
            # Official guidance recommends rapid retries because this service
            # may intermittently lose responses across boards.
            for attempt in range(1, 9):
                req = SetMcInputSource.Request()
                req.action.value = action
                req.input_source.name = "raicom_tasks"
                # The physical RC continuously owns priority 80 even with
                # neutral sticks. Priority 81 is limited to this attended
                # trial and expires one second after publishing stops.
                req.input_source.priority = 81
                req.input_source.timeout = 1000
                req.request.header.stamp = self.get_clock().now().to_msg()
                future = self._src.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=0.4)
                if not future.done() or future.result() is None:
                    self.get_logger().warning(
                        f"二开运动输入源{label}第{attempt}/8次无响应"
                    )
                    continue
                result = future.result().response
                code = result.header.code
                state = result.state.value
                ok = code == 0
                self.get_logger().info(
                    f"二开运动输入源{label}第{attempt}/8次: "
                    f"code={code}, state={state}, {'成功' if ok else '失败'}"
                )
                if ok:
                    return True
                # ADD code=1 normally means the source already exists; let the
                # caller fall back to MODIFY rather than hammering ADD.
                if action == 1001:
                    break
            return False

        # MC may keep input sources across task processes. Refresh an existing
        # source first; only a freshly started MC needs ADD.
        self._has_source = request(1002, "刷新(MODIFY)") or request(1001, "注册(ADD)")
        if not self._has_source:
            # ADD may reveal an existing source after all MODIFY responses were
            # lost. Give MODIFY one final retry sequence in that case.
            self._has_source = request(1002, "再次刷新(MODIFY)")
        if not self._has_source:
            self.get_logger().error("二开运动输入源刷新/注册失败")
        return self._has_source

    def set_mode(self, mode: str) -> bool:
        if not self._mc.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("SetMcAction 服务不可用")
            return False
        for attempt in range(1, 7):
            req = SetMcAction.Request()
            req.header = RequestHeader()
            req.header.stamp = self.get_clock().now().to_msg()
            req.source = "raicom_tasks"
            req.command = McActionCommand()
            req.command.action_desc = mode
            future = self._mc.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done() or future.result() is None:
                self.get_logger().warning(f"切换模式 {mode} 第{attempt}次无响应")
            else:
                result = future.result().response
                code = getattr(result.header, "code", -1)
                state = getattr(result.status, "value", -1)
                if code == 0 and state == 1:
                    self.get_logger().info(
                        f"切换模式 {mode}: code={code}, state={state}, 成功"
                    )
                    return True
                self.get_logger().warning(
                    f"切换模式 {mode} 第{attempt}次未就绪: code={code}, state={state}"
                )
            if attempt < 6:
                time.sleep(0.8)
        self.get_logger().error(f"切换模式 {mode} 重试后仍失败")
        return False

    def verify_control_source(self) -> bool:
        if not self._current_src.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("GetCurrentInputSource服务不可用")
            return False
        future = self._current_src.call_async(GetCurrentInputSource.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("查询当前运动输入源无响应")
            return False
        result = future.result()
        source = result.input_source.name
        self.get_logger().info(f"当前运动输入源: {source}")
        if result.response.header.code != 0 or source != "raicom_tasks":
            self.get_logger().error("MC未选择raicom_tasks，拒绝发送非零速度")
            return False
        return True

    def publish_velocity(
        self, forward: float = 0.0, lateral: float = 0.0, angular: float = 0.0
    ):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "raicom_tasks"
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)
        self._vel_pub.publish(msg)

    def stop(self):
        # 连续发送数帧零速，避免单帧丢失后机器人继续运动。
        if not rclpy.ok():
            return
        for _ in range(5):
            self.publish_velocity()
            rclpy.spin_once(self, timeout_sec=0.02)

    def _fresh_pose_or_stop(self):
        pose, age, _ = self._current_pose()
        if pose is None or age > self.POSE_MAX_AGE_S:
            self.stop()
            raise RuntimeError(f"定位丢失或过期（age={age:.2f}s），已停车")
        return pose

    def rotate_to(self, target_yaw: float, timeout_s: float = 25.0) -> bool:
        deadline = time.monotonic() + timeout_s
        stable_since = None
        next_progress_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self._fresh_pose_or_stop()
            error = normalize_angle(target_yaw - yaw)
            if abs(error) <= self.YAW_TOLERANCE_RAD:
                if stable_since is None:
                    stable_since = time.monotonic()
                self.publish_velocity()
                if time.monotonic() - stable_since >= 0.35:
                    self.stop()
                    self.get_logger().info(
                        f"转向完成: x={x:.3f}, y={y:.3f}, "
                        f"yaw={math.degrees(yaw):.1f}°, error={math.degrees(error):.1f}°"
                    )
                    return True
                time.sleep(0.02)
                continue
            stable_since = None
            # 真机在接近目标时必须允许低速收敛；固定 0.10rad/s 会反复跨过
            # 角度容差，表现为原地左右摆动直至超时。
            turn_limit = MAX_ANGULAR
            angular = max(-turn_limit, min(turn_limit, 0.9 * error))
            minimum_angular = min(0.045, MAX_ANGULAR)
            if abs(angular) < minimum_angular:
                angular = math.copysign(minimum_angular, angular)
            now = time.monotonic()
            if now >= next_progress_log:
                self.get_logger().info(
                    f"转向闭环: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°, "
                    f"error={math.degrees(error):.1f}°, cmd_angular={angular:.3f}"
                )
                next_progress_log = now + 1.0
            self.publish_velocity(angular=angular)
            time.sleep(0.02)
        self.stop()
        return False

    def drive_to(self, target_x: float, target_y: float, timeout_s: float = 35.0) -> bool:
        deadline = time.monotonic() + timeout_s
        next_progress_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self._fresh_pose_or_stop()
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            if distance <= self.POSITION_TOLERANCE_M:
                self.stop()
                self.get_logger().info(f"已进入目标容差: distance={distance:.3f}m")
                return True

            heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
            # 朝向偏差过大时原地转向，避免走弧线越界或碰撞。
            if abs(heading_error) > math.radians(25.0):
                forward = 0.0
            else:
                forward = min(MAX_FORWARD, max(min(0.06, MAX_FORWARD), 0.55 * distance))
                forward *= max(0.25, math.cos(heading_error))
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, 1.4 * heading_error))
            now = time.monotonic()
            if now >= next_progress_log:
                self.get_logger().info(
                    f"导航闭环: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°, "
                    f"distance={distance:.3f}m, heading_error={math.degrees(heading_error):.1f}°, "
                    f"cmd=({forward:.3f},0.000,{angular:.3f})"
                )
                next_progress_log = now + 2.0
            self.publish_velocity(forward=forward, angular=angular)
            time.sleep(0.02)
        self.stop()
        return False

    def dock_at(
        self, target_x: float, target_y: float, target_yaw: float, timeout_s: float = 40.0
    ) -> bool:
        """保持评分朝向并通过倒退进入交互区。"""
        deadline = time.monotonic() + timeout_s
        stable_since = None
        next_progress_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw = self._fresh_pose_or_stop()
            dx, dy = target_x - x, target_y - y
            distance = math.hypot(dx, dy)
            yaw_error = normalize_angle(target_yaw - yaw)

            if (
                distance <= self.DOCK_POSITION_TOLERANCE_M
                and abs(yaw_error) <= self.YAW_TOLERANCE_RAD
            ):
                if stable_since is None:
                    stable_since = time.monotonic()
                # 一进入圆心附近就发零速，让双脚落稳；若落脚漂出容差再恢复闭环。
                self.publish_velocity()
                if time.monotonic() - stable_since >= 0.5:
                    self.stop()
                    self.get_logger().info(
                        f"最终就位稳定: distance={distance:.3f}m, yaw_error={math.degrees(yaw_error):.1f}°"
                    )
                    return True
                time.sleep(0.02)
                continue
            else:
                stable_since = None

            # 控制目标放在圆心后方，避免倒退速度在圆边缘过早衰减进入步态死区；
            # 成功条件仍按真实圆心 target_x/target_y 计算。
            # Put the control point beyond the circle in the reverse-motion
            # direction.  This works for both Task1 (-90°) and returning home
            # (+90°), unlike a fixed world-Y offset.
            body_lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            body_forward_to_target = math.cos(yaw) * dx + math.sin(yaw) * dy
            # This chassis has no lateral command.  Once the reverse line has
            # drifted sideways, rotating in place here only makes the feet
            # shuffle beside the scoring circle.  Stop and let run_task1 move
            # back to an outside alignment point before trying again.
            if not SUPPORTS_LATERAL and abs(body_lateral) > 0.14:
                self.stop()
                self.get_logger().warning(
                    f"末段横向误差过大，退出本次停靠: lateral={body_lateral:.3f}m"
                )
                return False
            # Before a reverse docking target, body_forward_to_target is
            # negative.  Once it becomes positive the torso has passed the
            # target plane; never continue reversing away from the circle.
            if not SUPPORTS_LATERAL and body_forward_to_target > 0.03:
                self.stop()
                self.get_logger().error(
                    f"已越过末段目标平面，强制停车: forward_error={body_forward_to_target:.3f}m"
                )
                return False
            if SUPPORTS_LATERAL:
                control_dx = target_x - math.cos(target_yaw) * self.DOCK_CONTROL_OVERSHOOT_M - x
                control_dy = target_y - math.sin(target_yaw) * self.DOCK_CONTROL_OVERSHOOT_M - y
                body_forward = math.cos(yaw) * control_dx + math.sin(yaw) * control_dy
                forward = max(-MAX_FORWARD, min(MAX_FORWARD, 0.75 * body_forward))
                lateral = max(-MAX_LATERAL, min(MAX_LATERAL, 0.75 * body_lateral))
                steering_error = yaw_error
            else:
                # The pre-turn point has already compensated the real X2's
                # turn drift.  Mixing reverse translation with small angular
                # corrections makes the real gait alternate its feet in place
                # near the circle.  Keep the two operations exclusive: reverse
                # straight inside a generous yaw deadband, and only rotate
                # while stopped if the heading has genuinely escaped it.
                steering_error = yaw_error
                if abs(yaw_error) > math.radians(12.0):
                    self.stop()
                    self.get_logger().warning(
                        f"末段朝向漂移过大，退出本次停靠: yaw_error={math.degrees(yaw_error):.1f}°"
                    )
                    return False
                else:
                    # The real X2 needs a decisive command to take full reverse
                    # steps.  Tapering below 0.15 m/s makes it shuffle in place.
                    forward = -MAX_FORWARD
                lateral = 0.0
            if not SUPPORTS_LATERAL and forward != 0.0:
                angular = 0.0
            else:
                angular_limit = min(MAX_ANGULAR, 0.08)
                angular = max(-angular_limit, min(angular_limit, 1.2 * steering_error))
            now = time.monotonic()
            if now >= next_progress_log:
                self.get_logger().info(
                    "末段闭环: "
                    f"x={x:.3f}, y={y:.3f}, distance={distance:.3f}m, "
                    f"yaw_error={math.degrees(yaw_error):.1f}°, "
                    f"cmd=({forward:.3f}, {lateral:.3f}, {angular:.3f})"
                )
                next_progress_log = now + 2.0
            self.publish_velocity(forward=forward, lateral=lateral, angular=angular)
            time.sleep(0.02)
        self.stop()
        return False

    def align_reverse_line(
        self, target_x: float, target_y: float, target_yaw: float,
        lateral_tolerance_m: float = 0.06,
    ) -> bool:
        """Move outside the circle until straight reverse can reach the target."""
        x, y, yaw = self._fresh_pose_or_stop()
        dx, dy = target_x - x, target_y - y
        lateral = -math.sin(target_yaw) * dx + math.cos(target_yaw) * dy
        if abs(lateral) <= lateral_tolerance_m:
            self.get_logger().info(
                f"停靠轴线已对齐: lateral={lateral:.3f}m"
            )
            return True

        forward_to_target = math.cos(target_yaw) * dx + math.sin(target_yaw) * dy
        # Stay well outside the scoring circle.  Project onto the desired
        # reverse line while retaining a useful approach distance.
        approach_distance = max(0.80, min(1.40, -forward_to_target))
        align_x = target_x + math.cos(target_yaw) * approach_distance
        align_y = target_y + math.sin(target_yaw) * approach_distance
        self.get_logger().warning(
            f"转向产生横移 {lateral:.3f}m；圈外重对停靠轴线 "
            f"({align_x:.3f}, {align_y:.3f})"
        )
        if not self.drive_to(align_x, align_y, timeout_s=30.0):
            return False
        return self.rotate_to(target_yaw, timeout_s=25.0)

    def run_task1(self) -> bool:
        if not self.wait_for_pose() or not self.register_source():
            return False
        target_x, target_y = ZONES["交互区-I"]
        approach_x, approach_y = map(
            float, NAVIGATION_PROFILE["task1_turn_point_xy"]
        )
        final_yaw = math.radians(float(NAVIGATION_PROFILE["task1_final_yaw_deg"]))
        self.get_logger().info(
            "正式 Task1：前往交互区II附近提前转向，随后直线倒入交互区I；"
            f"转向点=({approach_x:.3f}, {approach_y:.3f})，"
            f"最终目标=({target_x:.3f}, {target_y:.3f}, {math.degrees(final_yaw):.2f}°)"
        )
        if not self.set_mode("STAND_DEFAULT"):
            self.get_logger().error("无法切换至稳定站立模式")
            return False
        time.sleep(2.0)
        if not self.set_mode("LOCOMOTION_DEFAULT"):
            self.get_logger().error("无法切换至行走模式")
            return False
        self.stop()
        # MC reports the last active source here.  A newly registered source
        # publishing zero velocity is not selected until its first non-zero
        # command, so pre-motion source verification would deadlock forever.
        try:
            self.get_logger().info("阶段1：前往交互区 II 附近的提前转向点")
            if not self.drive_to(approach_x, approach_y, timeout_s=45.0):
                self.get_logger().error("到达提前转向点超时")
                return False
            self.get_logger().info("阶段2：原地转到标定的交互朝向")
            if not self.rotate_to(final_yaw, timeout_s=30.0):
                self.get_logger().error("最终朝向调整超时")
                return False
            self.get_logger().info("阶段3：保持朝向，以完整后退步直线倒入交互区 I")
            if not self.dock_at(target_x, target_y, final_yaw, timeout_s=25.0):
                self.get_logger().error("直线倒入交互区 I 失败，已停车")
                return False
            x, y, yaw = self._fresh_pose_or_stop()
            position_error = math.hypot(target_x - x, target_y - y)
            yaw_error = abs(normalize_angle(final_yaw - yaw))
            self.get_logger().info(
                f"最终核验: position_error={position_error:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.2f}°"
            )
            if position_error > 0.15:
                self.get_logger().error("倒车后位置误差超过15cm，已停车")
                return False
            self.set_mode("STAND_DEFAULT")
            self.get_logger().info("Task1 完成：已在15cm容差内停车并面向交互区 II")
            return True
        finally:
            self.stop()
            self.set_mode("STAND_DEFAULT")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--wait", action="store_true", help="调试时等待 Enter；比赛运行不得使用")
    args = parser.parse_args()
    rclpy.init()
    agent = Task1Agent()
    if args.wait:
        input("按 Enter 开始 Task1...")
    success = False
    try:
        success = agent.run_task1()
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        agent.get_logger().error(str(exc))
    except Exception as exc:  # 现场必须留下未知异常，而不是静默退出。
        import traceback

        agent.get_logger().error(f"Task1 未处理异常: {type(exc).__name__}: {exc}")
        traceback.print_exc()
    finally:
        if rclpy.ok():
            agent.stop()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
