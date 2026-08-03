#!/usr/bin/env python3
"""Hide simulation-only claw actuators from the 14-joint closed-source MC."""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import JointStateArray

ARM_NAMES = {
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_yaw", "wrist_pitch", "wrist_roll")
}


class ArmStateRelay(Node):
    def __init__(self):
        super().__init__("raicom_arm_state_relay")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(
            JointStateArray, "/aima/hal/joint/arm/state", qos)
        self.create_subscription(
            JointStateArray, "/aima/sim/joint/arm/state_raw", self.relay, qos)

    def relay(self, incoming):
        outgoing = JointStateArray()
        outgoing.header = incoming.header
        outgoing.joints = [joint for joint in incoming.joints
                           if joint.name in ARM_NAMES]
        if len(outgoing.joints) == 14:
            self.publisher.publish(outgoing)


def main():
    rclpy.init()
    node = ArmStateRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
