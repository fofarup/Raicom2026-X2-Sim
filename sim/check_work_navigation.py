#!/usr/bin/env python3
"""Navigate from the current live pose to the work zone without a reset."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy

from competition_node import CompetitionNode
from core.navigator import WORK_ZONE


def main():
    args = SimpleNamespace(sim=True, auto_prepare=False, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    rclpy.init()
    node = CompetitionNode(args)
    ok = False
    try:
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        start = node.motion.pose
        ok = (start is not None and node.input_source.register()
              and node.mode.set("LD"))
        if ok:
            ok = node.navigator.goto(
                *WORK_ZONE, speed=0.35, timeout=240.0, tolerance=0.40)
        final = node.motion.pose
        error = (math.hypot(final[0] - WORK_ZONE[0], final[1] - WORK_ZONE[1])
                 if final else math.inf)
        print(f"work_navigation_ok={ok} start={start} final={final} "
              f"target_error={error:.3f}")
    finally:
        node.motion.stop(1.0)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok and error < 0.40 else 1)


if __name__ == "__main__":
    main()
