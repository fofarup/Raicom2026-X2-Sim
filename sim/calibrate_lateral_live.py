#!/usr/bin/env python3
"""Measure lateral CPG direction from the current pose without a reset."""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy
from rclpy.node import Node

from core.locomotion import InputSource, MotionController
from core.mode_switch import ModeSwitch
from core.navigator import Navigator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lateral", type=float, default=0.50)
    parser.add_argument("--duration", type=float, default=2.0)
    opts = parser.parse_args()
    rclpy.init()
    node = Node("raicom_lateral_live")
    motion = MotionController(node, "raicom_lateral_live")
    Navigator(node, motion, sim=True)
    source = InputSource(node, "raicom_lateral_live", 50)
    mode = ModeSwitch(node, lambda: motion.publish(0.02))
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
    before = motion.pose
    ok = before is not None and source.register() and mode.set("LD")
    deadline = time.monotonic() + opts.duration
    while ok and time.monotonic() < deadline:
        motion.publish(0.0, 0.0, opts.lateral)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.02)
    motion.stop(1.0)
    after = motion.pose
    dx, dy = after[0] - before[0], after[1] - before[1]
    dyaw = math.atan2(math.sin(after[3] - before[3]),
                      math.cos(after[3] - before[3]))
    print(f"lateral={opts.lateral:+.2f} before={before} after={after} "
          f"delta=({dx:+.3f},{dy:+.3f}) yaw_delta_deg={math.degrees(dyaw):+.1f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
