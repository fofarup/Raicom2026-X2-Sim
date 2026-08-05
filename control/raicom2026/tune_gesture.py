#!/usr/bin/env python3
"""交互式手臂关节调试——键盘调角度，按 S 保存。

控制键：
  1/2/3/4/5/6/7 — 选左臂关节 (sp/sr/sy/el/wy/wp/wr)
  q/w/e/r/t/y/u — 选右臂关节
  ↑/↓ — 当前关节 ±0.05 rad (~3°)
  PgUp/PgDn — 当前关节 ±0.20 rad (~11°)
  S — 保存当前姿态到文件
  R — 回到 READY
  Z — 全部归零
  Q — 退出
"""

import os
import sys
import time
sys.path.insert(0, os.path.dirname(__file__))

import rclpy
from rclpy.node import Node
from core.mode_switch import ModeSwitch
from core.grasp import GraspController, ALL_ARM_JOINTS

JOINT_NAMES = ["sp(肩俯仰)", "sr(肩横滚)", "sy(肩偏航)",
               "el(肘)", "wy(腕偏航)", "wp(腕俯仰)", "wr(腕横滚)"]

LEFT_KEYS = "1234567"
RIGHT_KEYS = "qwertyu"


def print_state(angles, selected, label=""):
    """打印当前关节角。"""
    print(f"\033[2J\033[H")  # 清屏
    print(f"╔══════════════════════════════════════════╗")
    print(f"║  交互式手臂调参  {label:<20}║")
    print(f"╠══════════════════════════════════════════╣")
    print(f"║ 选中关节: {JOINT_NAMES[selected%7]} "
          f"({'左臂' if selected < 7 else '右臂'})"
          f" 当前值: {angles[selected]:.3f} rad ({round(angles[selected]*57.3)}°) ║")
    print(f"╠══════════════════════════════════════════╣")

    for side, offset, prefix in [("左臂", 0, "L"), ("右臂", 7, "R")]:
        print(f"║ {side}:", end="")
        for i in range(7):
            idx = offset + i
            marker = ">" if idx == selected else " "
            v = angles[idx]
            print(f" {prefix}{JOINT_NAMES[i][:2]}={marker}{v:+.3f}", end="")
        print(" ║")

    print(f"╠══════════════════════════════════════════╣")
    print(f"║ L:1-7 R:q-w-e-r-t-y-u  ↑↓:±0.05  Pg:±0.2║")
    print(f"║ S:保存  R:READY  Z:归零  Q:退出        ║")
    print(f"╚══════════════════════════════════════════╝")


def apply_angles(grasp, angles, duration=0.3):
    """发送关节角到手臂。"""
    grasp.move_arm(angles, duration=duration)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("tune_gesture")
    mode = ModeSwitch(node)
    grasp = GraspController(node, sim=args.sim)

    # READY pose
    angles = [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0,
              -0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]

    # 切 US 模式
    print("切换到 US 模式...")
    if not mode.set("US"):
        print("US 模式失败")
        node.destroy_node()
        rclpy.shutdown()
        return

    selected = 0  # 当前选中的关节索引
    label = ""

    try:
        import tty
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)

        print_state(angles, selected)
        while True:
            ch = sys.stdin.read(1)

            if ch == "\x1b":  # ESC序列
                ch2 = sys.stdin.read(2)
                if ch2 == "[A":  # ↑
                    angles[selected] += 0.05
                elif ch2 == "[B":  # ↓
                    angles[selected] -= 0.05
                elif ch2 == "[5":  # PgUp
                    sys.stdin.read(1)
                    angles[selected] += 0.20
                elif ch2 == "[6":  # PgDn
                    sys.stdin.read(1)
                    angles[selected] -= 0.20
            elif ch in LEFT_KEYS:
                selected = LEFT_KEYS.index(ch)
            elif ch in RIGHT_KEYS:
                selected = 7 + RIGHT_KEYS.index(ch)
            elif ch in "sS":
                # 保存
                name = input("\n动作名(如 敬礼): ").strip() or "saved"
                ts = time.strftime("%H%M%S")
                fname = f"/tmp/gesture_{name}_{ts}.txt"
                with open(fname, "w") as f:
                    f.write(f"# {name}\n")
                    f.write(f"# saved at {time.ctime()}\n")
                    f.write(f"[{', '.join(f'{v:.3f}' for v in angles)}]\n")
                label = f"已保存: {fname}"
            elif ch in "rR":
                angles = [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0,
                          -0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
                label = "回到 READY"
            elif ch in "zZ":
                angles = [0.0] * 14
                label = "全部归零"
            elif ch in "qQ" or ord(ch) == 3:  # q/Q/Ctrl-C
                break

            # Clamp to limits
            limits = [
                -3.08, -0.061, -2.556, -2.356, -2.556, -0.558, -1.571,
                -3.08, -2.993, -2.556, -2.356, -2.556, -0.558, -0.724,
            ]
            uppers = [
                2.04, 2.993, 2.556, 0.0, 2.556, 0.558, 0.724,
                2.04, 0.061, 2.556, 0.0, 2.556, 0.558, 1.571,
            ]
            angles[selected] = max(limits[selected],
                                   min(uppers[selected], angles[selected]))

            apply_angles(grasp, angles)
            print_state(angles, selected, label)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        apply_angles(grasp, angles, duration=0.5)
        node.destroy_node()
        rclpy.shutdown()
        print("\n退出。保存的文件在 /tmp/")


if __name__ == "__main__":
    main()
