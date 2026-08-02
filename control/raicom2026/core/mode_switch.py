"""机器人运动模式切换。"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import RequestHeader
from aimdk_msgs.srv import SetMcAction


MODES = {
    "PD": "PASSIVE_DEFAULT",
    "DD": "DAMPING_DEFAULT",
    "JD": "JOINT_DEFAULT",
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
}


class ModeSwitch:
    def __init__(self, node: Node):
        self._node = node
        self._client = node.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")

    def set(self, mode: str) -> bool:
        if mode not in MODES:
            self._node.get_logger().error(f"未知模式: {mode}")
            return False
        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("SetMcAction 服务不可用")
            return False
        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "raicom2026"
        req.command.action_desc = MODES[mode]
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        ok = future.done() and future.result() is not None
        if ok:
            self._node.get_logger().info(f"模式: {mode}")
        return ok
