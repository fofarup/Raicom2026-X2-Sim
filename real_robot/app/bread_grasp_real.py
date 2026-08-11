#!/usr/bin/env python3
"""RK4 right-claw bread-neck grasp; never publishes a base command."""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from aimdk_msgs.msg import (
    JointStateArray, McActionCommand, MessageHeader, RequestHeader,
    UpperBodyCommandArray,
)
from aimdk_msgs.srv import SetMcAction
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from robot_profile import load_robot_profile

IK_SRC = Path(__file__).resolve().parent / "vendor" / "x2_ik_sdk" / "src"
sys.path.insert(0, str(IK_SRC))
from x2_ik_sdk import ArmSide, X2ArmIKSolver, X2IKConfig  # noqa: E402
from x2_ik_sdk.config import ARM_POS_ORDER  # noqa: E402


class BreadGrasp(Node):
    RATE = 50.0
    MAX_SPEED = 0.25

    def __init__(self) -> None:
        super().__init__("raicom_bread_grasp_real")
        self.arm = None
        self.arm_at = 0.0
        self.sequence = 0
        self.pub = self.create_publisher(UpperBodyCommandArray, "/mc/upper_body_command", 10)
        self.create_subscription(
            JointStateArray, "/aima/hal/joint/arm/state", self._arm_cb,
            qos_profile_sensor_data,
        )
        self.mode = self.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        self.solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())

    def _arm_cb(self, msg: JointStateArray) -> None:
        values = {j.name: float(j.position) for j in msg.joints}
        if all(name in values for name in ARM_POS_ORDER):
            self.arm = [values[name] for name in ARM_POS_ORDER]
            self.arm_at = time.monotonic()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_arm(self, seconds: float = 4.0):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.arm is not None and time.monotonic() - self.arm_at < 0.3:
                return list(self.arm)
        return None

    def set_mode(self, name: str) -> bool:
        if not self.mode.wait_for_service(timeout_sec=4.0):
            return False
        for _ in range(4):
            req = SetMcAction.Request()
            req.header = RequestHeader()
            req.header.stamp = self.get_clock().now().to_msg()
            req.source = "remote_teleop_pc"
            req.command = McActionCommand()
            req.command.action_desc = name
            future = self.mode.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result():
                result = future.result().response
                if result.header.code == 0 and result.status.value == 1:
                    self.get_logger().info(f"模式 {name} 成功")
                    return True
            time.sleep(0.5)
        return False

    def publish(self, arm, right_open: bool) -> None:
        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self.sequence
        self.sequence += 1
        msg.source = "remote_teleop_pc"
        msg.hand_sub_mode = UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = list(map(float, arm))
        msg.hand_pos = [0.0, 1.0 if right_open else 0.0]
        self.pub.publish(msg)

    def hold(self, arm, right_open: bool, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish(arm, right_open)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.RATE)

    def move(self, target, right_open: bool, label: str) -> bool:
        start = self.wait_arm()
        if start is None:
            self.get_logger().error(f"{label}: 无手臂反馈")
            return False
        target_xyz = np.asarray(self.solver.fk_xyz(ArmSide.RIGHT,target))
        command_target = list(target)
        for attempt in range(3):
            duration = max(
                1.5,
                max(abs(a-b) for a,b in zip(start,command_target)) / self.MAX_SPEED,
            )
            steps = math.ceil(duration * self.RATE)
            self.get_logger().info(
                f"{label} 第{attempt+1}次: {duration:.2f}s, {steps}步"
            )
            for i in range(1, steps + 1):
                alpha = i / steps
                command = [a + (b-a)*alpha for a,b in zip(start,command_target)]
                self.publish(command, right_open)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(1.0 / self.RATE)
            self.hold(command_target, right_open, 1.2)
            actual = self.wait_arm(1.0)
            if actual is None:
                return False
            joint_error = max(abs(a-b) for a,b in zip(actual[7:],target[7:]))
            actual_xyz = np.asarray(self.solver.fk_xyz(ArmSide.RIGHT,actual))
            ee_error = float(np.linalg.norm(target_xyz-actual_xyz))
            self.get_logger().info(
                f"{label}反馈: joint={joint_error:.3f}rad ee={ee_error:.3f}m "
                f"actual_xyz={[round(float(v),3) for v in actual_xyz]}"
            )
            if joint_error <= 0.10 and ee_error <= 0.035:
                return True
            if attempt == 2:
                break
            # MC on this real X2 has a repeatable steady-state joint offset.
            # Compensate only the right arm, clamp every retry to 0.16 rad, and
            # retain fresh feedback as the next interpolation start.
            corrected = list(command_target)
            for idx in range(7, 14):
                delta = max(-0.16, min(0.16, target[idx] - actual[idx]))
                corrected[idx] += delta
            command_target = corrected
            start = actual
        return False

    def run(self) -> bool:
        current = self.wait_arm()
        if current is None:
            self.get_logger().error("没有手臂状态，取消")
            return False
        # OmniPicker local +Z points toward the bread; fingers close along pelvis Y.
        rotation = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
        targets = [
            ("安全预抓取", [0.295,-0.075,0.335]),
            ("靠近面包袋颈部", [0.355,-0.070,0.322]),
            ("夹持后抬升", [0.335,-0.080,0.395]),
        ]
        plans = []
        seed = current
        for label, xyz in targets:
            result = self.solver.solve_pose(ArmSide.RIGHT, xyz, rotation, seed)
            if not result.success or result.error_norm > 0.004:
                self.get_logger().error(f"{label} IK失败: {result.error_norm:.4f}m")
                return False
            plans.append((label, result.arm_pos))
            seed = result.arm_pos

        if not self.set_mode("STAND_DEFAULT"):
            return False
        time.sleep(1.0)
        if not self.set_mode("UPPERBODY_REMOTE_SPLIT"):
            return False
        takeover = self.wait_arm()
        if takeover is None:
            return False
        self.hold(takeover, True, 1.0)
        try:
            if not self.move(plans[0][1], True, plans[0][0]):
                return False
            if not self.move(plans[1][1], True, plans[1][0]):
                return False
            self.get_logger().info("闭合右夹爪")
            self.hold(plans[1][1], False, 2.0)
            if not self.move(plans[2][1], False, plans[2][0]):
                return False
            self.get_logger().info("面包抓取动作完成")
            self.hold(plans[2][1], False, 1.0)
            return True
        finally:
            # Do not command legs; SD safely resumes normal whole-body holding.
            self.set_mode("STAND_DEFAULT")


def main() -> int:
    profile = load_robot_profile()
    if os.environ.get("RAICOM_CONFIRM_REAL_ROBOT") != "YES":
        print(
            "拒绝执行：设置 RAICOM_CONFIRM_REAL_ROBOT=YES 前不会启动真机手臂控制",
            file=sys.stderr,
        )
        return 2
    if os.environ.get("RAICOM_CONFIRM_EXPERIMENTAL_ARM_MOTION") != "YES":
        print(
            "拒绝执行：该固定轨迹仅供有人持急停的实验；"
            "确认后设置 RAICOM_CONFIRM_EXPERIMENTAL_ARM_MOTION=YES",
            file=sys.stderr,
        )
        return 2
    if not bool(profile["grasp"].get("calibrated", False)):
        print("拒绝执行：grasp.calibrated=false", file=sys.stderr)
        return 2
    rclpy.init()
    node = BreadGrasp()
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
