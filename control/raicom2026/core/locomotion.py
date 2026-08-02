"""速度控制 + 输入源注册 + 航向修正。"""

import math
import time
from typing import Optional, Tuple

from rclpy.node import Node
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class InputSource:
    def __init__(self, node: Node, name: str = "raicom2026", priority: int = 40):
        self._node = node
        self._client = node.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
        self._name = name
        self._priority = priority

    def register(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=10.0):
            return False
        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self._name
        req.input_source.priority = self._priority
        req.input_source.timeout = 1000
        future = self._client.call_async(req)
        import rclpy
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        return future.done() and future.result() is not None


class MotionController:
    TOPIC = "/aima/mc/locomotion/velocity"

    def __init__(self, node: Node, source_name: str = "raicom2026"):
        self._node = node
        self._source = source_name
        self._pub = node.create_publisher(McLocomotionVelocity, self.TOPIC, 10)
        self._position = None  # (x, y, z, yaw)

    def update_pose(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        self._position = (x, y, z, yaw)

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        if self._position is None:
            return None
        return (self._position[0], self._position[1], self._position[2])

    @property
    def yaw(self) -> Optional[float]:
        if self._position is None:
            return None
        return self._position[3]

    def publish(self, forward: float, angular: float = 0.0):
        msg = McLocomotionVelocity()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.source = self._source
        msg.forward_velocity = forward
        msg.lateral_velocity = 0.0
        msg.angular_velocity = angular
        self._pub.publish(msg)

    def stop(self, duration: float = 1.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish(0.0)
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(0.02)

    def move_toward(
        self, target_x: float, target_y: float,
        speed: float = 0.15, tolerance: float = 0.15,
        timeout: float = 30.0,
    ) -> bool:
        self._node.get_logger().info(f"移动至 ({target_x:.2f}, {target_y:.2f})")
        deadline = time.monotonic() + timeout
        import rclpy
        cnt = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            cnt += 1
            if self.position is None:
                time.sleep(0.01)
                continue
            px, py, _ = self.position
            dist = math.hypot(target_x - px, target_y - py)
            if dist < tolerance:
                self._node.get_logger().info("已到达")
                self.stop(1.0)
                return True
            target_yaw = math.atan2(target_y - py, target_x - px)
            yaw_err = target_yaw - (self.yaw or target_yaw)
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            angular = max(-0.5, min(0.5, yaw_err * 0.8))
            fwd = speed * min(1.0, dist / 0.5)
            if cnt % 15 == 0:
                self._node.get_logger().info(
                    f"  dist={dist:.2f} yaw_err={math.degrees(yaw_err):.0f}°")
            self.publish(fwd, angular)
            time.sleep(0.02)
        self._node.get_logger().warn("移动超时！")
        self.stop(1.0)
        return False
