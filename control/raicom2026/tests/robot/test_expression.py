#!/usr/bin/env python3
"""真机表情测试 —— 逐个显示比赛五种表情。

用法（在 PC2 真机上）：
  python3 test_expression.py
  python3 test_expression.py --all       # 自动遍历所有表情
  python3 test_expression.py --emotion 快乐  # 显示指定表情

依赖：ROS 2 Humble + AimDK SDK
"""

import sys
import time
import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import CommonRequest
from aimdk_msgs.srv import PlayEmoji

EMOTIONS = {
    "快乐": 90,
    "悲伤": 110,
    "愤怒": 180,
    "睡觉": 80,
    "充电": 220,
    "疑惑": 130,
    "平静-卖萌": 30,
    "平静": 10,
}


class ExpressionTester(Node):
    def __init__(self):
        super().__init__("test_expression")
        self._client = self.create_client(PlayEmoji, "/aimdk_5Fmsgs/srv/PlayEmoji")
        self.get_logger().info("表情测试节点就绪")

    def show(self, name: str) -> bool:
        eid = EMOTIONS.get(name)
        if eid is None:
            self.get_logger().error(f"未知表情: {name}, 可选: {list(EMOTIONS.keys())}")
            return False

        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("PlayEmoji 服务不可用，确认 MC 已启动")
            return False

        req = PlayEmoji.Request()
        req.header = CommonRequest()
        req.emotion_id = eid
        req.mode = 1          # EMOTION_MODE_ONCE
        req.priority = 50

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        ok = future.done() and future.result() is not None
        if ok:
            self.get_logger().info(f"✅ {name} (id={eid})")
        else:
            self.get_logger().error(f"❌ {name} 失败")
        return ok


def main():
    rclpy.init()
    tester = ExpressionTester()

    if "--all" in sys.argv:
        print("=== 遍历全部表情 ===")
        for name in EMOTIONS:
            print(f"\n>>> {name}")
            tester.show(name)
            time.sleep(2.0)
        print("\n完成。")
    elif "--emotion" in sys.argv:
        idx = sys.argv.index("--emotion")
        name = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else "快乐"
        tester.show(name)
    else:
        print("可用表情:", ", ".join(EMOTIONS.keys()))
        print()
        while True:
            try:
                name = input("输入表情名(q退出): ").strip()
                if name.lower() == "q":
                    break
                tester.show(name)
            except (EOFError, KeyboardInterrupt):
                break
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
