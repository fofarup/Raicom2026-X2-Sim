#!/usr/bin/env python3
"""Closed-loop in-place rotation acceptance using the real simulated gait."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))

import rclpy
from rclpy.node import Node

from core.locomotion import InputSource, MotionController
from core.mode_switch import ModeSwitch
from core.navigator import Navigator


def main():
    rclpy.init()
    node = Node("raicom_rotation_acceptance")
    motion = MotionController(node, "raicom_rotation_test")
    Navigator(node, motion, sim=True)
    source = InputSource(node, "raicom_rotation_test", 50)
    mode = ModeSwitch(node, lambda: motion.publish(0.02))
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
    before = motion.pose
    ok = (before is not None and source.register() and mode.set("LD"))
    target = None if before is None else before[3] - math.pi / 2
    if ok:
        ok = motion.rotate_to(target, timeout=70.0)
    after = motion.pose
    drift = math.inf if before is None or after is None else math.dist(before[:2], after[:2])
    error = math.inf if target is None or after is None else abs(math.atan2(
        math.sin(target - after[3]), math.cos(target - after[3])))
    print(f"rotation_ok={ok} drift={drift:.4f} yaw_error={error:.4f} "
          f"before={before} after={after}")
    motion.stop(1.0)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok and drift <= 0.15 else 1)


if __name__ == "__main__":
    main()
