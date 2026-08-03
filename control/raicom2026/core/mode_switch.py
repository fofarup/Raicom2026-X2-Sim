"""机器人运动模式切换。"""

import time
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import CommonState, RequestHeader
from aimdk_msgs.srv import GetMcAction, SetMcAction


MODES = {
    "PD": "PASSIVE_DEFAULT",
    "DD": "DAMPING_DEFAULT",
    "JD": "JOINT_DEFAULT",
    "SD": "STAND_DEFAULT",
    "LD": "LOCOMOTION_DEFAULT",
    "US": "UPPERBODY_REMOTE_SPLIT",
}


class ModeSwitch:
    def __init__(self, node: Node, locomotion_heartbeat=None):
        self._node = node
        self._locomotion_heartbeat = locomotion_heartbeat
        self._client = node.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        self._get_client = node.create_client(
            GetMcAction, "/aimdk_5Fmsgs/srv/GetMcAction")

    def wait(self, mode: str, timeout: float = 10.0) -> bool:
        """Wait until *mode* is the MC's active RUNNING action."""
        if mode not in MODES:
            return False
        action_desc = MODES[mode]
        if not self._get_client.wait_for_service(timeout_sec=2.0):
            return False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (action_desc == MODES["LD"] and
                    self._locomotion_heartbeat is not None):
                self._locomotion_heartbeat()
            future = self._get_client.call_async(GetMcAction.Request())
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=0.5)
            response = future.result() if future.done() else None
            if (response is not None and response.header.code == 0 and
                    response.info.action_desc == action_desc and
                    response.info.status.value == 100):
                return True
            time.sleep(0.1)
        return False

    def request(self, mode: str) -> bool:
        """Request a mode transition without waiting for RUNNING.

        MuJoCo recovery needs this split operation: request SD, reset the
        simulation state, and only then wait for SD to become stable.
        """
        if mode not in MODES:
            self._node.get_logger().error(f"未知模式: {mode}")
            return False
        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("SetMcAction 服务不可用")
            return False
        req = SetMcAction.Request()
        req.header = RequestHeader()
        # MC 模式服务使用官方控制源 rc；它与速度消息的输入源注册彼此独立。
        req.source = "rc"
        req.command.action_desc = MODES[mode]
        if mode == "LD" and self._locomotion_heartbeat is not None:
            self._locomotion_heartbeat()
        response = None
        for attempt in range(8):
            req.header.stamp = self._node.get_clock().now().to_msg()
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=0.4)
            response = future.result() if future.done() else None
            if (response is not None and
                    response.response.status.value == CommonState.SUCCESS):
                break
            time.sleep(0.15)
        if response is None or response.response.status.value != CommonState.SUCCESS:
            message = response.response.message if response is not None else "无响应"
            self._node.get_logger().error(f"模式 {mode} 请求失败: {message}")
            return False
        return True

    def set(self, mode: str) -> bool:
        # SetMcAction rejects requesting LOCOMOTION_DEFAULT while that same
        # action is already RUNNING.  Treat the target state as idempotent.
        if self.wait(mode, timeout=0.4):
            self._node.get_logger().info(
                f"模式: {mode} ({MODES[mode]}) RUNNING")
            return True
        # The MC may acknowledge LD->US while keeping locomotion in a long
        # stopping transition. Going through the documented stand action makes
        # the upper-body split transition deterministic after a long walk.
        if mode in ("LD", "US") and not self.wait("SD", timeout=0.4):
            if not self.request("SD") or not self.wait("SD", timeout=15.0):
                self._node.get_logger().error(f"进入 {mode} 前未能切换到 SD")
                return False
        if not self.request(mode):
            return False
        wait_timeout = 45.0 if mode == "US" else 15.0
        if not self.wait(mode, timeout=wait_timeout):
            self._node.get_logger().error(f"模式 {mode} 未进入 RUNNING")
            return False
        self._node.get_logger().info(f"模式: {mode} ({MODES[mode]}) RUNNING")
        return True
