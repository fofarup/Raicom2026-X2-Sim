"""表情控制模块。仿真用 emoji 日志，真机通过 PlayEmoji 服务控制 X2 屏幕。"""

from rclpy.node import Node
from aimdk_msgs.msg import CommonRequest
from aimdk_msgs.srv import PlayEmoji

# 比赛五种表情 → PlayEmoji 枚举值
EMOTION_MAP = {
    "快乐": 90,      # EMOTION_EYE_HAPPY
    "悲伤": 110,     # EMOTION_EYE_SAD
    "愤怒": 180,     # EMOTION_EYE_ANGRY
    "睡觉": 80,      # EMOTION_EYE_SLEEPY
    "充电": 220,     # EMOTION_EYE_CHARGE
    "疑惑": 130,     # EMOTION_EYE_CONFUSE
    "平静-卖萌": 30, # EMOTION_IDLE_CUTE_1
    "平静": 10,      # EMOTION_IDLE_CALM_1
}

EXPRESSIONS = list(EMOTION_MAP.keys())
SERVICE = "/aimdk_5Fmsgs/srv/PlayEmoji"


class ExpressionController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        if not sim:
            self._client = node.create_client(PlayEmoji, SERVICE)

    def show(self, expression: str):
        if self._sim:
            emoji_map = {
                "快乐": "😊", "悲伤": "😢", "愤怒": "😡",
                "睡觉": "😴", "充电": "🔋", "疑惑": "🤔",
                "平静-卖萌": "😇", "平静": "😐",
            }
            self._node.get_logger().info(
                f"[表情] {emoji_map.get(expression, '😶')} {expression}")
            return

        emotion_id = EMOTION_MAP.get(expression)
        if emotion_id is None:
            self._node.get_logger().error(f"未知表情: {expression}")
            return

        if not self._client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error("PlayEmoji 服务不可用")
            return

        req = PlayEmoji.Request()
        req.header = CommonRequest()
        req.emotion_id = emotion_id
        req.mode = 1          # EMOTION_MODE_ONCE
        req.priority = 50

        import rclpy
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            self._node.get_logger().info(
                f"[表情] {expression} (emotion_id={emotion_id}) ✅")
        else:
            self._node.get_logger().error(f"表情设置失败: {expression}")
