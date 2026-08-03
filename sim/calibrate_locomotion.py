#!/usr/bin/env python3
"""Measure one reset-isolated CPG velocity command in MuJoCo."""
import argparse
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy
from competition_node import CompetitionNode


def angle_delta(a, b):
    return math.atan2(math.sin(b - a), math.cos(b - a))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.30)
    parser.add_argument("--lateral", type=float, default=0.0)
    parser.add_argument("--angular", type=float, default=-0.052)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--pulse", type=float, default=0.0,
                        help="active seconds followed by one second zero command")
    opts = parser.parse_args()
    rclpy.init()
    args = SimpleNamespace(sim=True, auto_prepare=True, auto_start=True,
                           number_image="number_01.png", expression="快乐",
                           gesture="挥右手", need="口渴", hand="right")
    node = CompetitionNode(args)
    if not node.prepare():
        raise SystemExit("FAIL: setup")
    before = node.motion.pose
    min_height = before[2]
    started = time.monotonic()
    deadline = started + opts.duration
    while time.monotonic() < deadline:
        elapsed = time.monotonic() - started
        active = not opts.pulse or elapsed % (opts.pulse + 1.0) < opts.pulse
        node.motion.publish(opts.forward if active else 0.0,
                            opts.angular if active else 0.0,
                            opts.lateral if active else 0.0)
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.motion.pose:
            min_height = min(min_height, node.motion.pose[2])
        time.sleep(0.02)
    node.motion.stop(1.0)
    after = node.motion.pose
    print(f"forward={opts.forward:+.2f} lateral={opts.lateral:+.2f} "
          f"angular={opts.angular:+.3f} duration={opts.duration:.1f} "
          f"yaw_delta={angle_delta(before[3], after[3]):+.3f} "
          f"travel={math.dist(before[:2], after[:2]):.3f} "
          f"min_height={min_height:.3f} before={before} after={after}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
