#!/usr/bin/env python3
"""Conservative X2 real-robot straight-line calibration utility."""

import argparse
import math
import signal
import time

import rclpy
from aimdk_msgs.msg import McActionCommand, McLocomotionVelocity, MessageHeader, RequestHeader
from aimdk_msgs.srv import GetCurrentInputSource, SetMcAction, SetMcInputSource
from nav_msgs.msg import Odometry
from rclpy.node import Node


SOURCE = "raicom_calibration"


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MotionCalibration(Node):
    def __init__(self):
        super().__init__("raicom_motion_calibration")
        self.pose = None
        self.command = (0.0, 0.0, 0.0)
        self.velocity_pub = self.create_publisher(
            McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10
        )
        self.create_subscription(Odometry, "/slam/lidar_odom", self._pose_cb, 10)
        self.source_client = self.create_client(
            SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource"
        )
        self.action_client = self.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )
        self.current_source_client = self.create_client(
            GetCurrentInputSource, "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
        )
        self.create_timer(0.05, self._publish_velocity)

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def _publish_velocity(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = SOURCE
        msg.forward_velocity, msg.lateral_velocity, msg.angular_velocity = self.command
        self.velocity_pub.publish(msg)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_pose(self, timeout=5.0):
        self.spin_for(0.2)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def register_source(self):
        if not self.source_client.wait_for_service(timeout_sec=5.0):
            return False
        for action in (1002, 1001):
            req = SetMcInputSource.Request()
            req.action.value = action
            req.input_source.name = SOURCE
            req.input_source.priority = 40
            req.input_source.timeout = 1000
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.source_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.done() and future.result() and future.result().response.header.code == 0:
                return True
        return False

    def set_mode(self, name):
        if not self.action_client.wait_for_service(timeout_sec=5.0):
            return False
        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.header.stamp = self.get_clock().now().to_msg()
        req.source = SOURCE
        req.command = McActionCommand()
        req.command.action_desc = name
        future = self.action_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result():
            return False
        result = future.result().response
        return result.header.code == 0 and result.status.value == 1

    def current_source(self):
        if not self.current_source_client.wait_for_service(timeout_sec=3.0):
            return None
        future = self.current_source_client.call_async(GetCurrentInputSource.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or not future.result():
            return None
        result = future.result()
        if result.response.header.code != 0:
            return None
        return result.input_source.name

    def stop(self, seconds=1.0):
        self.command = (0.0, 0.0, 0.0)
        self.spin_for(seconds)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    if not 0.0 < args.speed <= 0.20 or not 0.1 <= args.duration <= 3.0:
        raise SystemExit("安全限制: speed必须为(0,0.20]，duration必须为[0.1,3.0]")

    rclpy.init()
    node = MotionCalibration()
    interrupted = False

    def request_stop(*_):
        nonlocal interrupted
        interrupted = True
        node.command = (0.0, 0.0, 0.0)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    start = None
    try:
        if not node.wait_pose():
            raise RuntimeError("未收到/slam/lidar_odom，拒绝运动")
        start = node.pose
        print(f"START x={start[0]:.4f} y={start[1]:.4f} yaw={math.degrees(start[2]):.2f}deg", flush=True)
        if not node.register_source():
            raise RuntimeError("标定输入源注册失败")
        node.stop(0.5)
        if not node.set_mode("LOCOMOTION_DEFAULT"):
            raise RuntimeError("切换LOCOMOTION_DEFAULT失败")
        node.stop(1.0)
        selected_source = node.current_source()
        print(f"CURRENT_SOURCE {selected_source or 'unknown'}", flush=True)
        if selected_source != SOURCE:
            raise RuntimeError(
                f"MC当前输入源为{selected_source or 'unknown'}，不是{SOURCE}，拒绝发送非零速度"
            )
        print(f"MOVE speed={args.speed:.3f}m/s duration={args.duration:.2f}s", flush=True)
        node.command = (args.speed, 0.0, 0.0)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and not interrupted and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.stop(1.0)
    except Exception as exc:
        print(f"FAIL {exc}", flush=True)
    finally:
        node.stop(1.0)
        node.set_mode("STAND_DEFAULT")
        node.stop(0.5)
        end = node.pose
        if start and end:
            dx, dy = end[0] - start[0], end[1] - start[1]
            print(
                f"END x={end[0]:.4f} y={end[1]:.4f} yaw={math.degrees(end[2]):.2f}deg "
                f"delta=({dx:.4f},{dy:.4f}) distance={math.hypot(dx, dy):.4f}m",
                flush=True,
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
