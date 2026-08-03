#!/usr/bin/env python3
"""Reset-isolated calibration of alternating-step yaw control."""
import argparse
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy

from competition_node import CompetitionNode


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--turn-degrees", type=float, default=-90.0)
    parser.add_argument("--forward", type=float, default=0.10)
    parser.add_argument("--phase", type=float, default=0.60)
    parser.add_argument("--timeout", type=float, default=30.0)
    opts = parser.parse_args()
    args = SimpleNamespace(sim=True, auto_prepare=True, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    rclpy.init()
    node = CompetitionNode(args)
    ok = False
    try:
        if not node.prepare():
            raise SystemExit("FAIL: setup")
        before = node.motion.pose
        target = before[3] + math.radians(opts.turn_degrees)
        started = time.monotonic()
        deadline = started + opts.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            error = angle_error(target, node.motion.yaw)
            if abs(error) <= math.radians(10):
                ok = True
                break
            phase_index = int((time.monotonic() - started) / opts.phase)
            forward = opts.forward if phase_index % 2 == 0 else -opts.forward
            angular = max(-0.30, min(0.30, 0.30 * error))
            node.motion.publish(forward, angular)
            time.sleep(0.02)
        node.motion.stop(1.0)
        after = node.motion.pose
        drift = math.dist(before[:2], after[:2])
        final_error = abs(angle_error(target, after[3]))
        print(f"turn_ok={ok} phase={opts.phase:.2f} forward={opts.forward:.2f} "
              f"drift={drift:.3f} yaw_error_deg={math.degrees(final_error):.1f} "
              f"before={before} after={after}")
    finally:
        node.motion.stop(0.5)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok and drift <= 0.20 else 1)


if __name__ == "__main__":
    main()
