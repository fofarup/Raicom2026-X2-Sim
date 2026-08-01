#!/usr/bin/env python3
"""任务5：自主移动（仅限全国总决赛）— 交互区-I → 作业区。

评分点：
  1) 到达指定区域，机身投影覆盖目标位置点
  2) 自主导航得分，遥控得 0 分

实现：
  - 基于里程计的自主导航 + 航向修正
  - 途经点路径规划（避免直穿障碍）
  - 到达后语音播报确认

场地坐标参考：
  交互区-I: (0, 1.0)    作业区: (1.5, -1.4)
"""

import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node

from x2_utils import (
    SimConfig,
    ModeSwitch,
    InputSource,
    MotionController,
    WaypointNavigator,
    SpeechController,
    set_ready,
    init_robot,
)


class Task5Autonomous(Node):
    def __init__(self, sim: bool = True):
        super().__init__("task5_autonomous")
        self._sim = sim

        tools = init_robot(self, sim)
        self.mode = tools["mode"]
        self.input_src = tools["input_src"]
        self.motion = tools["motion"]
        self.speech = tools["speech"]
        self._navigator = WaypointNavigator(self.motion)

        from nav_msgs.msg import Odometry
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Odometry, "/aima/hal/odom/state", self.motion.on_odom, qos_profile=qos
        )

    def plan_path(
        self, start: tuple, goal: tuple
    ) -> list:
        """规划从 start 到 goal 的途经点。

        简单实现：分段直线，避免直接穿墙。
        实际比赛可用 A* 等算法替代。
        """
        sx, sy = start
        gx, gy = goal
        waypoints = []

        # 如果横向距离大，先走一段横向再纵向（L 型）
        dx = gx - sx
        dy = gy - sy

        mid_x = sx + dx * 0.5
        mid_y = sy + dy * 0.5

        # 插入中间点（可根据实际场地障碍调整）
        waypoints.append((mid_x, sy + dy * 0.2))
        waypoints.append((mid_x, sy + dy * 0.6))
        waypoints.append(goal)

        return waypoints

    def run(self, goal: tuple = None):
        if goal is None:
            goal = SimConfig.WORK_ZONE

        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  任务5：自主移动\n"
            f"  交互区-I → 作业区\n"
            f"  目标: ({goal[0]:.2f}, {goal[1]:.2f})\n"
            f"{'='*50}"
        )

        if not set_ready(self.mode, self.input_src):
            self.get_logger().error("无法进入运动模式")
            return

        # 等待获取当前位置
        rclpy.spin_once(self, timeout_sec=1.0)
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.motion.position is not None:
                break
            time.sleep(0.05)

        pos = self.motion.position
        if pos is None:
            self.get_logger().error("无法获取里程计数据！")
            return

        start = (pos[0], pos[1])
        self.get_logger().info(f"当前位置: ({start[0]:.2f}, {start[1]:.2f})")

        # 规划路径
        waypoints = self.plan_path(start, goal)
        self.get_logger().info(f"途经点: {waypoints}")

        # 执行导航
        ok = self._navigator.go(waypoints, speed=0.18, timeout_per_point=45.0)

        if ok:
            self.speech.say("已到达作业区。")
            self.get_logger().info("✅ 自主移动完成！")
        else:
            self.get_logger().error("导航失败")

        self.motion.stop(1.0)
        self.get_logger().info("任务5结束。")


def main():
    parser = argparse.ArgumentParser(description="任务5：自主移动")
    parser.add_argument("--goal-x", type=float, default=SimConfig.WORK_ZONE[0])
    parser.add_argument("--goal-y", type=float, default=SimConfig.WORK_ZONE[1])
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Task5Autonomous(sim=args.sim)
    try:
        node.run(goal=(args.goal_x, args.goal_y))
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.motion.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
