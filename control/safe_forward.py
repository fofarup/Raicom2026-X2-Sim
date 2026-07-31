#!/usr/bin/env python3
"""X2 仿真低速前进/安全停止测试。

运行前必须先让机器人进入 SD 并在 MuJoCo 中 Reset，然后切换到 LD。
"""

import argparse
import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import McLocomotionVelocity
from aimdk_msgs.srv import SetMcInputSource


TOPIC = "/aima/mc/locomotion/velocity"
SERVICE = "/aimdk_5Fmsgs/srv/SetMcInputSource"
SOURCE_NAME = "x2_biao_safe_demo"


class SafeMotion(Node):
    def __init__(self) -> None:
        super().__init__("x2_biao_safe_motion")
        self.publisher = self.create_publisher(McLocomotionVelocity, TOPIC, 10)
        self.client = self.create_client(SetMcInputSource, SERVICE)

    def register(self) -> None:
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"MC 服务不可用：{SERVICE}")

        request = SetMcInputSource.Request()
        request.action.value = 1001
        request.input_source.name = SOURCE_NAME
        request.input_source.priority = 40
        request.input_source.timeout = 1000

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("注册 MC 输入源超时。")

        self.get_logger().info("MC 输入源注册完成。")

    def publish_velocity(self, forward: float) -> None:
        message = McLocomotionVelocity()
        message.header.stamp = self.get_clock().now().to_msg()
        message.source = SOURCE_NAME
        message.forward_velocity = forward
        message.lateral_velocity = 0.0
        message.angular_velocity = 0.0
        self.publisher.publish(message)

    def move_for(self, speed: float, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_velocity(speed)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def stop(self, duration: float = 1.5) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish_velocity(0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self.get_logger().info("已连续发送零速度停止指令。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="X2 仿真安全低速前进测试")
    parser.add_argument("--speed", type=float, default=0.12, help="前进速度 m/s")
    parser.add_argument("--duration", type=float, default=2.0, help="运动时间 s")
    parser.add_argument("--stop-only", action="store_true", help="只发送停止指令")
    args = parser.parse_args()

    if not 0.0 <= args.speed <= 0.25:
        parser.error("--speed 必须处于 0.0~0.25 m/s")
    if not 0.1 <= args.duration <= 10.0:
        parser.error("--duration 必须处于 0.1~10.0 s")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = SafeMotion()
    try:
        node.register()
        if not args.stop_only:
            node.get_logger().info(
                f"开始低速前进：{args.speed:.2f} m/s，持续 {args.duration:.1f} s"
            )
            node.move_for(args.speed, args.duration)
    except KeyboardInterrupt:
        node.get_logger().warning("收到 Ctrl+C，立即停止。")
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
