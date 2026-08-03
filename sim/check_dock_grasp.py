#!/usr/bin/env python3
"""Re-dock from a nearby live pose, then run physical grasp acceptance."""
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
    ok = False
    try:
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        # This focused acceptance deliberately preserves the pose reached by
        # the preceding navigation check. Register a fresh velocity source,
        # but do not issue the competition pre-start GUI Reset.
        ok = (node.motion.pose is not None
              and node.input_source.register()
              and node.mode.set("LD"))
        if ok:
            ok = node.navigator.dock_for_grasp(need.object_world_xyz, "right")
        if ok:
            ok = node.mode.set("US")
            for _ in range(40):
                rclpy.spin_once(node, timeout_sec=0.025)
            pose = node.motion.pose
            observed = node.grasp.object_position(need.object_name)
            world = ((observed[0], observed[1], need.object_world_xyz[2])
                     if observed else need.object_world_xyz)
            target = world_to_base(world, pose)
            print(f"redock_pose={pose} target_base={target}")
        if ok:
            ok = node.grasp.grasp_and_lift(
                "right", target, object_name=need.object_name)
        if ok:
            ok = node.grasp.hold_grip("right", 5.0)
        print(f"dock_grasp_ok={ok}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
