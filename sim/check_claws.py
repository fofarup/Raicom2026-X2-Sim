#!/usr/bin/env python3
"""Exercise both simulated physical claws and verify state feedback."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "control" / "raicom2026"))
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import JointStateArray

from core.grasp import GraspController


def main():
    rclpy.init()
    node = Node("raicom_claw_check")
    positions = {}

    def receive(msg):
        for joint in msg.joints:
            positions[joint.name] = joint.position

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(JointStateArray, "/aima/sim/joint/arm/state_raw", receive, qos)
    grasp = GraspController(node, sim=True)
    failures = []
    for side in ("left", "right"):
        name = "L_claw_joint" if side == "left" else "R_claw_joint"
        grasp.grip(side, 1.0, duration=2.5)
        opened = positions.get(name)
        grasp.grip(side, 0.0, duration=2.5)
        closed = positions.get(name)
        print(f"{side}: open={opened} close={closed}")
        if opened is None or closed is None or opened < 0.025 or closed > 0.01:
            failures.append(side)
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        raise SystemExit(f"FAIL: physical claw feedback did not move: {failures}")
    print("PASS: both physical claws opened and closed")


if __name__ == "__main__":
    main()
