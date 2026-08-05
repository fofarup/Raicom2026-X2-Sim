#!/usr/bin/env python3
"""手臂手势测试脚本。逐一测试 5 个比赛动作。

运行：
  python3 test_gesture.py
  python3 test_gesture.py --sim        # 仿真模式
  python3 test_gesture.py --all        # 自动循环全部
"""

import argparse
import os
import sys
import time
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node

from core.mode_switch import ModeSwitch
from core.grasp import GraspController
from core.gesture import GestureController, POSES

GESTURE_LIST = list(POSES.keys())  # ["挥左手","挥右手","左手敬礼","右手敬礼","双手打叉"]


def main():
    parser = argparse.ArgumentParser(description="手臂手势测试")
    parser.add_argument("--sim", action="store_true", default=True, help="仿真模式")
    parser.add_argument("--all", action="store_true", help="自动循环全部，不交互")
    parser.add_argument("--gesture", choices=GESTURE_LIST, help="只测指定动作")
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_gesture")

    mode = ModeSwitch(node)
    grasp = GraspController(node, sim=args.sim)
    gesture = GestureController(grasp)

    print("\n" + "=" * 50)
    print("  手臂手势测试")
    print("  " + ("仿真模式" if args.sim else "真机模式"))
    print("=" * 50)

    # 切 US 模式（上体遥控，下体 MC 保持）
    print("\n>>> 切换到 US 模式...")
    if not mode.set("US"):
        print("❌ US 模式切换失败，检查 MC 是否运行")
        node.destroy_node()
        rclpy.shutdown()
        return
    print("✅ US 模式就绪")

    if args.gesture:
        todo = [args.gesture]
    elif args.all:
        todo = GESTURE_LIST
    else:
        # 交互式选择
        print("\n可选动作:")
        for i, name in enumerate(GESTURE_LIST, 1):
            print(f"  {i}. {name}")
        print("  0. 全部循环")
        print("  q. 退出")
        choice = input("\n选择: ").strip()

        if choice.lower() == "q":
            node.destroy_node()
            rclpy.shutdown()
            return
        elif choice == "0":
            todo = GESTURE_LIST
        elif choice.isdigit() and 1 <= int(choice) <= len(GESTURE_LIST):
            todo = [GESTURE_LIST[int(choice) - 1]]
        else:
            print("无效选择")
            node.destroy_node()
            rclpy.shutdown()
            return

    # 执行
    for name in todo:
        print(f"\n>>> 执行: {name}")
        start = time.monotonic()
        ok = gesture.perform(name)
        elapsed = time.monotonic() - start
        status = "✅" if ok else "❌"
        print(f"  {status} {name} ({elapsed:.1f}s)")

        if name != todo[-1]:
            time.sleep(1.0)  # 动作间短暂停顿

    # 回 READY
    print("\n>>> 恢复预备姿态...")
    gesture.return_to_ready()
    print("✅ 测试结束")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
