#!/usr/bin/env python3
"""Command one arm through the project controller and verify joint feedback."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy
from rclpy.node import Node

from core.grasp import GraspController, compose_arm_target
from core.mode_switch import ModeSwitch


def main():
    rclpy.init()
    node = Node("raicom_arm_motion_check")
    grasp = GraspController(node, sim=True)
    mode = ModeSwitch(node)
    if not mode.set("US"):
        raise SystemExit("FAIL: could not enter UPPERBODY_REMOTE_SPLIT")
    if not grasp.wait_for_arm_state():
        raise SystemExit("FAIL: no arm state")
    start = list(grasp._arm_positions)
    active = [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
    target = compose_arm_target(start, "left", active)
    grasp.move_arm(target, duration=2.0)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    final = list(grasp._arm_positions)
    error = max(abs(final[i] - target[i]) for i in range(7))
    travel = max(abs(final[i] - start[i]) for i in range(7))
    print(f"left_arm travel={travel:.4f} max_target_error={error:.4f}")
    node.destroy_node()
    rclpy.shutdown()
    if travel < 0.1 or error > 0.20:
        raise SystemExit("FAIL: arm command did not reach target")
    print("PASS: arm command reached feedback target")


if __name__ == "__main__":
    main()
