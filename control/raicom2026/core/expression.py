"""表情控制模块。仿真用 print，真机通过屏幕服务控制。"""

from rclpy.node import Node

EXPRESSIONS = ["悲伤", "睡觉", "愤怒", "快乐", "充电", "疑惑", "平静-卖萌", "平静"]


class ExpressionController:
    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim

    def show(self, expression: str):
        if self._sim:
            emoji_map = {
                "快乐": "😊", "悲伤": "😢", "愤怒": "😡",
                "睡觉": "😴", "充电": "🔋", "疑惑": "🤔",
                "平静-卖萌": "😇", "平静": "😐",
            }
            emoji = emoji_map.get(expression, "😶")
            self._node.get_logger().info(f"[表情] {emoji} {expression}")
        else:
            # TODO: 真机屏幕 API
            self._node.get_logger().info(f"设置表情: {expression}")
