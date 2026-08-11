#!/usr/bin/env python3
"""RAICOM 2026 official Task3: need reasoning and closed-loop work-zone navigation."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from task1_agent import (
    MAX_ANGULAR, MAX_FORWARD, MAX_LATERAL, NAVIGATION_PROFILE,
    SUPPORTS_LATERAL, Task1Agent, ZONES, normalize_angle,
)
from task3_needs import classify_need, run_demand_tests


class Task3SceneAgent(Task1Agent):
    WORK_POSITION_TOLERANCE_M = float(NAVIGATION_PROFILE["work_position_tolerance_m"])
    TABLE_POSITION = tuple(NAVIGATION_PROFILE["table_position_xy"])
    # Facing the table spreads the feet mainly along world Y.  This offset is a
    # footprint calibration value, not a change to the scoring-zone coordinate.
    FOOTPRINT_DOCK_OFFSET_Y_M = float(NAVIGATION_PROFILE["footprint_dock_offset_y_m"])
    FOOTPRINT_DOCK_TOLERANCE_M = float(
        NAVIGATION_PROFILE["footprint_dock_tolerance_m"]
    )
    PREALIGN_POSITION_TOLERANCE_M = float(
        NAVIGATION_PROFILE["prealign_position_tolerance_m"]
    )
    MAX_TABLE_REALIGN_ATTEMPTS = int(NAVIGATION_PROFILE["max_table_realign_attempts"])
    MAX_SAFE_ZONE_DISTANCE_M = float(NAVIGATION_PROFILE["max_safe_zone_distance_m"])
    REALIGN_LATERAL_TRIGGER_M = float(
        NAVIGATION_PROFILE["realign_lateral_trigger_m"]
    )
    MAX_UNCORRECTABLE_LATERAL_M = float(
        NAVIGATION_PROFILE["max_uncorrectable_lateral_m"]
    )

    def __init__(self):
        super().__init__(node_name="task3_scene_agent")
        # Task1Agent creates all proven localization/motion interfaces.  A distinct
        # logger message makes accidental legacy Task4 startup immediately visible.
        self.get_logger().info("Task3 场景服务：需求判断 + 作业区闭环导航")

    def navigate_to_work_zone(self, keep_locomotion: bool = False) -> bool:
        target_x, target_y = ZONES["作业区"]
        self.POSITION_TOLERANCE_M = self.WORK_POSITION_TOLERANCE_M
        self.get_logger().info(
            f"Task3 导航目标：作业区中心 ({target_x:.2f}, {target_y:.2f})"
        )
        if not self.wait_for_pose() or not self.register_source():
            return False
        if not self.set_mode("STAND_DEFAULT"):
            self.get_logger().error("无法确认 SD，取消导航")
            return False
        time.sleep(1.5)
        if not self.set_mode("LOCOMOTION_DEFAULT"):
            self.get_logger().error("无法切换 LD，取消导航")
            return False
        keep_active = False
        try:
            if not self.drive_to(target_x, target_y, timeout_s=45.0):
                self.get_logger().error("作业区导航超时")
                return False
            self.stop()
            pose, age, source = self._current_pose()
            if pose is None or age > self.POSE_MAX_AGE_S:
                self.get_logger().error("停车后定位无效")
                return False
            distance = ((pose[0] - target_x) ** 2 + (pose[1] - target_y) ** 2) ** 0.5
            self.get_logger().info(
                f"作业区到达[{source}]：x={pose[0]:.3f}, y={pose[1]:.3f}, distance={distance:.3f}m"
            )
            success = distance <= self.WORK_POSITION_TOLERANCE_M
            keep_active = success and keep_locomotion
            return success
        finally:
            self.stop()
            if not keep_active and not self.set_mode("STAND_DEFAULT"):
                self.get_logger().error("导航结束后切回 SD 失败")

    def _drive_to_with_tolerance(
        self, target_x: float, target_y: float, tolerance_m: float, timeout_s: float
    ) -> bool:
        """Use the proven drive controller with a temporary tighter tolerance."""
        original_tolerance = self.POSITION_TOLERANCE_M
        self.POSITION_TOLERANCE_M = tolerance_m
        try:
            return self.drive_to(target_x, target_y, timeout_s=timeout_s)
        finally:
            self.POSITION_TOLERANCE_M = original_tolerance

    def _pose_error(self, target_x: float, target_y: float, target_yaw: float):
        x, y, yaw = self._fresh_pose_or_stop()
        dx, dy = target_x - x, target_y - y
        return (
            x,
            y,
            yaw,
            math.hypot(dx, dy),
            -math.sin(yaw) * dx + math.cos(yaw) * dy,
            normalize_angle(target_yaw - yaw),
        )

    def _rotate_to_table(self, target_yaw: float, timeout_s: float = 20.0) -> bool:
        """Gentle settled turn used only for final table alignment."""
        deadline = time.monotonic() + timeout_s
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            _, _, yaw = self._fresh_pose_or_stop()
            error = normalize_angle(target_yaw - yaw)
            if abs(error) <= self.YAW_TOLERANCE_RAD:
                self.publish_velocity()
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.40:
                    self.stop()
                    return True
                time.sleep(0.02)
                continue
            stable_since = None
            limit = min(MAX_ANGULAR, 0.35 if abs(error) > math.radians(25.0) else 0.20)
            angular = max(-limit, min(limit, 0.9 * error))
            if abs(angular) < 0.08:
                angular = math.copysign(0.08, angular)
            self.publish_velocity(angular=angular)
            time.sleep(0.02)
        self.stop()
        return False

    def align_to_table(
        self, timeout_s: float = 30.0, locomotion_ready: bool = False
    ) -> bool:
        """Face the table, then reverse/advance back to the work-zone centre."""
        zone_x, zone_y = ZONES["作业区"]
        target_x = zone_x
        target_y = zone_y + self.FOOTPRINT_DOCK_OFFSET_Y_M
        table_x, table_y = self.TABLE_POSITION
        # Keep facing the physical table from the nominal zone centre; only the
        # footprint docking position is biased.
        target_yaw = math.atan2(table_y - zone_y, table_x - zone_x)
        self.get_logger().info(
            f"桌面对正目标：yaw={math.degrees(target_yaw):.1f}°，足迹补偿目标="
            f"({target_x:.2f},{target_y:.2f})，作业区圆心=({zone_x:.2f},{zone_y:.2f})"
        )
        if not locomotion_ready:
            if not self.wait_for_pose() or not self.register_source():
                return False
            if not self.set_mode("STAND_DEFAULT"):
                return False
            time.sleep(1.0)
            if not self.set_mode("LOCOMOTION_DEFAULT"):
                return False
        else:
            self.get_logger().info("沿用Task3导航LD，连续进入预对桌闭环")
        try:
            # First remove the arrival error with a tighter tolerance.  The old
            # sequence rotated immediately after the 0.10 m work-zone arrival;
            # turn drift then converted that residual into an uncorrectable
            # body-lateral error.
            self.get_logger().info(
                f"预对桌回中：先以 {self.PREALIGN_POSITION_TOLERANCE_M:.2f}m 容差到达足迹目标"
            )
            if not self._drive_to_with_tolerance(
                target_x,
                target_y,
                self.PREALIGN_POSITION_TOLERANCE_M,
                timeout_s=15.0,
            ):
                self.get_logger().error("预对桌回中超时")
                return False

            # A turn can translate the simulated biped.  If that creates too
            # much lateral error, make a bounded small re-positioning move and
            # face the table again instead of forcing an unsafe arm operation.
            for attempt in range(self.MAX_TABLE_REALIGN_ATTEMPTS + 1):
                if not self._rotate_to_table(target_yaw, timeout_s=20.0):
                    self.get_logger().error("面向桌子转向超时")
                    return False
                rclpy.spin_once(self, timeout_sec=0.05)
                x, y, _, distance, lateral, yaw_error = self._pose_error(
                    target_x, target_y, target_yaw
                )
                zone_distance = math.hypot(zone_x - x, zone_y - y)
                self.get_logger().info(
                    f"对桌检查[{attempt + 1}/{self.MAX_TABLE_REALIGN_ATTEMPTS + 1}]："
                    f"dock_distance={distance:.3f}m, zone_distance={zone_distance:.3f}m, "
                    f"lateral={lateral:.3f}m, yaw_error={math.degrees(yaw_error):.1f}°"
                )
                if zone_distance > self.MAX_SAFE_ZONE_DISTANCE_M:
                    self.get_logger().error(
                        f"转向后距作业区圆心 {zone_distance:.3f}m，超出安全修正范围"
                    )
                    return False
                if abs(lateral) <= self.REALIGN_LATERAL_TRIGGER_M:
                    break
                if attempt >= self.MAX_TABLE_REALIGN_ATTEMPTS:
                    self.get_logger().error(
                        f"经过 {self.MAX_TABLE_REALIGN_ATTEMPTS} 次修正，横向残差仍为 "
                        f"{lateral:.3f}m"
                    )
                    return False
                self.get_logger().warning(
                    f"横向残差 {lateral:.3f}m，执行第 {attempt + 1} 次小范围退出/重新定位"
                )
                if not self._drive_to_with_tolerance(
                    target_x,
                    target_y,
                    self.PREALIGN_POSITION_TOLERANCE_M,
                    timeout_s=12.0,
                ):
                    self.get_logger().error("小范围重新定位超时")
                    return False

            deadline = time.monotonic() + timeout_s
            stable_since = None
            next_log = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                x, y, yaw = self._fresh_pose_or_stop()
                dx, dy = target_x - x, target_y - y
                distance = math.hypot(dx, dy)
                zone_distance = math.hypot(zone_x - x, zone_y - y)
                yaw_error = normalize_angle(target_yaw - yaw)
                if (
                    distance <= self.FOOTPRINT_DOCK_TOLERANCE_M
                    and zone_distance <= 0.12
                    and abs(yaw_error) <= self.YAW_TOLERANCE_RAD
                ):
                    self.publish_velocity()
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.5:
                        self.stop()
                        self.get_logger().info(
                            f"桌面对正完成：x={x:.3f}, y={y:.3f}, dock_distance={distance:.3f}m, "
                            f"zone_distance={zone_distance:.3f}m, "
                            f"yaw_error={math.degrees(yaw_error):.1f}°"
                        )
                        return True
                    time.sleep(0.02)
                    continue
                stable_since = None

                # At the table-facing yaw, turn drift is predominantly along body X;
                # correct it with forward/reverse motion while holding heading.
                body_forward_error = math.cos(yaw) * dx + math.sin(yaw) * dy
                body_lateral_error = -math.sin(yaw) * dx + math.cos(yaw) * dy
                if (
                    not SUPPORTS_LATERAL
                    and abs(body_lateral_error) > self.MAX_UNCORRECTABLE_LATERAL_M
                ):
                    self.get_logger().error(
                        f"横向残差 {body_lateral_error:.3f}m 超出无横移底盘可修正范围，停车"
                    )
                    return False
                forward_limit = min(MAX_FORWARD, 0.16)
                forward = max(-forward_limit, min(forward_limit, 0.8 * body_forward_error))
                minimum_forward = min(0.06, forward_limit)
                if abs(forward) < minimum_forward and abs(body_forward_error) > 0.04:
                    forward = math.copysign(minimum_forward, body_forward_error)
                lateral = (
                    max(-MAX_LATERAL, min(MAX_LATERAL, 0.8 * body_lateral_error))
                    if SUPPORTS_LATERAL else 0.0
                )
                angular_limit = min(MAX_ANGULAR, 0.20)
                angular = max(-angular_limit, min(angular_limit, 1.2 * yaw_error))
                if time.monotonic() >= next_log:
                    self.get_logger().info(
                        f"对桌回中：dock_distance={distance:.3f}m, zone_distance={zone_distance:.3f}m, "
                        f"lateral={body_lateral_error:.3f}m, "
                        f"yaw_error={math.degrees(yaw_error):.1f}°"
                    )
                    next_log = time.monotonic() + 1.5
                self.publish_velocity(forward=forward, lateral=lateral, angular=angular)
                time.sleep(0.02)
            self.get_logger().error("对桌回中超时")
            return False
        finally:
            self.stop()
            if not self.set_mode("STAND_DEFAULT"):
                self.get_logger().error("对桌结束后切回 SD 失败")

    def navigate_and_align_to_table(self) -> bool:
        """Keep one LD session from interaction zone through final table alignment."""
        if not self.navigate_to_work_zone(keep_locomotion=True):
            return False
        return self.align_to_table(locomotion_ready=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-demands", action="store_true", help="只测需求判断，不启动 ROS")
    parser.add_argument("--navigate-only", action="store_true", help="只闭环导航到作业区")
    parser.add_argument("--align-table-only", action="store_true", help="从作业区内对正桌子并回中")
    parser.add_argument("--navigate-and-align", action="store_true", help="导航到作业区后对正桌子")
    parser.add_argument("--text", help="解析一条需求文本，不运动")
    args = parser.parse_args()
    if args.test_demands:
        raise SystemExit(0 if run_demand_tests() else 1)
    if args.text is not None:
        print(classify_need(args.text))
        return
    if not (args.navigate_only or args.align_table_only or args.navigate_and_align):
        parser.error("需明确指定导航/对桌、--text 或 --test-demands")

    rclpy.init()
    agent = Task3SceneAgent()
    success = False
    try:
        if args.align_table_only:
            success = agent.align_to_table()
        elif args.navigate_and_align:
            success = agent.navigate_and_align_to_table()
        else:
            success = agent.navigate_to_work_zone()
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        agent.get_logger().error(str(exc))
    finally:
        if rclpy.ok():
            agent.stop()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
