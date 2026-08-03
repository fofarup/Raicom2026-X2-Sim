#!/usr/bin/env python3
"""Physical grasp/lift acceptance from an already docked simulator pose."""

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))

import rclpy

from competition_node import CompetitionNode
from core.grasp import world_to_base
from core.scenario import NEEDS


def main():
    need = next(item for item in NEEDS if item.name == "口渴")
    args = SimpleNamespace(sim=True, auto_prepare=False, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    rclpy.init()
    node = CompetitionNode(args)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
    ok = node.mode.set("US")
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.025)
    pose = node.motion.pose
    ok = ok and pose is not None
    observed = node.grasp.object_position(need.object_name)
    object_world = ((observed[0], observed[1], need.object_world_xyz[2])
                    if observed else need.object_world_xyz)
    target = None if pose is None else world_to_base(object_world, pose)
    if ok:
        ok = node.grasp.grasp_and_lift(
            "right", target, object_name=need.object_name)
    if ok:
        ok = node.grasp.hold_grip("right", 5.0)
    print(f"grasp_only_ok={ok} pose={pose} target_base={target}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
