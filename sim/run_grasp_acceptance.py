#!/usr/bin/env python3
"""Navigate, physically grasp/lift, and hold one service object."""
import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy

from competition_node import CompetitionNode
from core.grasp import world_to_base
from core.navigator import WORK_ZONE
from core.scenario import NEEDS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--need", choices=[need.name for need in NEEDS], default="口渴")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    opts = parser.parse_args()
    need = next(item for item in NEEDS if item.name == opts.need)
    args = SimpleNamespace(sim=True, auto_prepare=True, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need=need.keywords[0], hand=opts.hand)
    rclpy.init()
    node = CompetitionNode(args)
    ok = False
    try:
        ok = node.prepare()
        if ok:
            ok = node.navigator.goto(
                *WORK_ZONE, speed=0.35, timeout=240, tolerance=0.40)
        if ok:
            ok = node.navigator.dock_for_grasp(need.object_world_xyz, opts.hand)
        if ok:
            ok = node.mode.set("US")
        if ok:
            for _ in range(40):
                rclpy.spin_once(node, timeout_sec=0.025)
            px, py, pz = node.motion.position
            observed = node.grasp.object_position(need.object_name)
            object_world = ((observed[0], observed[1], need.object_world_xyz[2])
                            if observed else need.object_world_xyz)
            target = world_to_base(object_world,
                                   (px, py, pz, node.motion.yaw))
            print(f"dock_pose={node.motion.pose} target_base={target}")
            ok = node.grasp.grasp_and_lift(
                opts.hand, target, object_name=need.object_name)
        if ok:
            ok = node.grasp.hold_grip(opts.hand, 5.0)
        print(f"grasp_ok={ok} need={need.name} object={need.object_name} hand={opts.hand}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
