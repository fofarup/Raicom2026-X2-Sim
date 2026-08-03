#!/usr/bin/env python3
"""Verify in-zone facing without resetting the current simulation pose."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy
from rclpy.node import Node

from core.locomotion import InputSource, MotionController
from core.mode_switch import ModeSwitch
from core.navigator import INTERACT_II, Navigator


def main():
    rclpy.init()
    node = Node("raicom_face_check")
    motion = MotionController(node, "raicom_face_check")
    nav = Navigator(node, motion, sim=True)
    mode = ModeSwitch(node, lambda: motion.publish(0.02, 0.0))
    source = InputSource(node, "raicom_face_check", priority=50)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    start = motion.pose
    ok = source.register() and mode.set("LD") and nav.face(*INTERACT_II)
    final = motion.pose
    motion.stop(1.0)
    drift = math.dist(start[:2], final[:2]) if start and final else math.inf
    yaw_error = abs(math.atan2(
        math.sin(-math.pi / 2 - final[3]), math.cos(-math.pi / 2 - final[3])))
    print(f"face_ok={ok} start={start} final={final} drift={drift:.3f} "
          f"yaw_error_deg={math.degrees(yaw_error):.1f}")
    node.destroy_node()
    rclpy.shutdown()
    if not ok or drift > 0.25 or yaw_error > math.radians(10):
        raise SystemExit("FAIL: facing acceptance")
    print("PASS: faced Interaction II while remaining in Interaction I")


if __name__ == "__main__":
    main()
