#!/usr/bin/env python3
"""Exercise a short behind-the-body precision translation."""

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
    node = Node("raicom_recenter_acceptance")
    motion = MotionController(node, "raicom_recenter_test")
    Navigator(node, motion, sim=True)
    source = InputSource(node, "raicom_recenter_test", 50)
    mode = ModeSwitch(node, lambda: motion.publish(0.02))
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.05)
    before = motion.pose
    target = (0.04, 1.54)
    ok = before is not None and source.register() and mode.set("LD")
    if ok:
        ok = motion.move_toward(*target, speed=0.35, tolerance=0.18,
                                timeout=20.0, obstacle_check=False)
    after = motion.pose
    error = math.inf if after is None else math.dist(after[:2], target)
    print(f"recenter_ok={ok} error={error:.4f} before={before} after={after}")
    motion.stop(1.0)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
