#!/usr/bin/env python3
"""Drive both scored navigation legs without running interaction/grasp tasks."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy

from competition_node import CompetitionNode
from core.navigator import INTERACT_I, INTERACT_II, WORK_ZONE


def main():
    args = SimpleNamespace(sim=True, auto_prepare=True, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    rclpy.init()
    node = CompetitionNode(args)
    ok = False
    try:
        ok = node.prepare()
        if ok:
            ok = node.navigator.goto(*INTERACT_I, speed=0.40, timeout=240)
        if ok:
            ok = node.navigator.face(*INTERACT_II)
        pose_i = node.motion.pose
        if ok:
            ok = node.navigator.goto(*WORK_ZONE, speed=0.35, timeout=240)
        pose_work = node.motion.pose
        print(f"navigation_ok={ok} interaction_pose={pose_i} work_pose={pose_work}")
    finally:
        node.motion.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
