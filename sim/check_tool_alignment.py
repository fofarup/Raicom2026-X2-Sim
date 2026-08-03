#!/usr/bin/env python3
"""Compare a commanded IK tool point with the physical MuJoCo finger centre."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))

import rclpy

from competition_node import CompetitionNode
from core.grasp import CLAW_TOOL_OFFSET, world_to_base


def main():
    args = SimpleNamespace(sim=True, auto_prepare=True, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    rclpy.init()
    node = CompetitionNode(args)
    target = [0.25, -0.20, 0.00]
    try:
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)
        ok = (node.prepare() and node.mode.set("US")
              and node.grasp._move_active_arm(
                  "right", target, CLAW_TOOL_OFFSET))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            if node.grasp._held_arm_target is not None:
                node.grasp._publish_upper_body(node.grasp._held_arm_target)
            rclpy.spin_once(node, timeout_sec=0.02)
        pose = node.motion.pose
        physical_world = node.grasp._claw_positions.get("right")
        physical_base = (world_to_base(physical_world, pose)
                         if pose and physical_world else None)
        error = ([physical_base[i] - target[i] for i in range(3)]
                 if physical_base else None)
        print(f"tool_alignment_ok={ok} pose={pose} target={target} "
              f"physical_base={physical_base} error={error}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok and error is not None else 1)


if __name__ == "__main__":
    main()
