"""抓取控制模块：IK 求解 + 手臂关节控制 + 夹爪控制。

依赖：x2_ik_sdk（Pinocchio IK 求解器）
"""

import os
import sys
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import JointCommandArray, JointCommand, HandCommandArray, HandCommand, HandType, MessageHeader

# IK SDK 路径
_IK_SDK_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "ik_sdk")
if _IK_SDK_PATH not in sys.path:
    sys.path.insert(0, _IK_SDK_PATH)

ARM_TOPIC = "/aima/hal/joint/arm/command"
HAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_ARM = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
]
RIGHT_ARM = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
]
ALL_ARM_JOINTS = LEFT_ARM + RIGHT_ARM


class GraspController:
    """IK + 手臂 + 夹爪一体化抓取控制。"""

    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        self._ik_solver = None

        # 手臂 publisher
        self._arm_pub = node.create_publisher(JointCommandArray, ARM_TOPIC, 10)
        # 夹爪 publisher
        hand_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._hand_pub = node.create_publisher(HandCommandArray, HAND_TOPIC, hand_qos)

    @property
    def ik(self):
        """延迟加载 IK solver（避免在没有 pin 的环境下导入失败）。"""
        if self._ik_solver is None:
            try:
                from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
                self._ik_solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
                self._node.get_logger().info("IK Solver 已加载")
            except ImportError as e:
                self._node.get_logger().warn(f"IK SDK 不可用: {e}")
        return self._ik_solver

    def solve_ik(self, side: str, target_xyz: list):
        """求解手臂 IK。side: 'left' 或 'right'。"""
        if self._sim:
            self._node.get_logger().info(f"[IK] {side} arm → {target_xyz}")
            # 仿真返回 ready pose 的近似值
            if side == "left":
                return [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
            else:
                return [-0.35, -0.45, 0.0, -1.0, 0.0, 0.15, 0.0]

        if self.ik is None:
            self._node.get_logger().error("IK Solver 不可用")
            return None

        try:
            from x2_ik_sdk import ArmSide
            side_enum = ArmSide.LEFT if side == "left" else ArmSide.RIGHT
            arm_pos = list(self.ik.ready_arm_pos())
            result = self.ik.solve_position(side_enum, target_xyz, arm_pos)
            if result.success:
                self._node.get_logger().info(f"IK 求解成功 error={result.error_norm:.4f}")
                return result.active_arm
            else:
                self._node.get_logger().warn(f"IK 未收敛: {result.message}")
                return None
        except Exception as e:
            self._node.get_logger().error(f"IK 求解失败: {e}")
            return None

    def move_arm(self, arm_pos: list, steps: int = 30, interval: float = 0.03):
        """移动手臂到目标关节角度。arm_pos[14] = left 7 + right 7。"""
        if self._sim:
            self._node.get_logger().info(f"[手臂] 移动到 {arm_pos[:3]}...")
            time.sleep(1.0)
            return

        for step in range(1, steps + 1):
            t = step / steps
            msg = JointCommandArray()
            msg.header = MessageHeader()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            for name, target in zip(ALL_ARM_JOINTS, arm_pos):
                cmd = JointCommand()
                cmd.name = name
                cmd.position = float(target * t)
                cmd.velocity = 0.1
                msg.joints.append(cmd)
            self._arm_pub.publish(msg)
            time.sleep(interval)

    def grip(self, hand: str, position: float):
        """控制夹爪。hand: 'left'/'right'，position: 0(闭合)~1(打开)。"""
        if self._sim:
            state = "闭合" if position < 0.5 else "打开"
            self._node.get_logger().info(f"[夹爪] {hand} {state}")
            return

        msg = HandCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        cmd = HandCommand()
        cmd.name = "left_claw_joint" if hand == "left" else "right_claw_joint"
        cmd.position = float(position)
        cmd.velocity = 0.3

        if hand == "left":
            msg.left_hand_type = HandType(value=0x2)
            msg.left_hands.append(cmd)
            msg.right_hand_type = HandType(value=0x0)
        else:
            msg.right_hand_type = HandType(value=0x2)
            msg.right_hands.append(cmd)
            msg.left_hand_type = HandType(value=0x0)

        self._hand_pub.publish(msg)

    def grasp_object(self, side: str, target_xyz: list, place_xyz: list = None):
        """完整抓取流程：IK → 移臂 → 抓 → 放。"""
        # 1. IK 求解
        arm_pos = self.solve_ik(side, target_xyz)
        if arm_pos is None:
            return False

        # 2. 张开夹爪
        self.grip(side, 1.0)
        time.sleep(0.5)

        # 3. 移臂到目标
        full_pos = list(arm_pos)
        # TODO: 正确填充 full_pos[14]
        self.move_arm(full_pos)

        # 4. 闭合夹爪抓取
        time.sleep(0.3)
        self.grip(side, 0.0)
        time.sleep(0.5)
        self._node.get_logger().info("已抓取")

        # 5. 放置（如果指定）
        if place_xyz:
            place_pos = self.solve_ik(side, place_xyz)
            if place_pos:
                self.move_arm(list(place_pos))
                time.sleep(0.3)
                self.grip(side, 1.0)
                self._node.get_logger().info("已放置")

        return True
