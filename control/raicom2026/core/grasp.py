"""双臂 IK、关节轨迹与 OmniPicker 物理抓取流程。"""
from __future__ import annotations

import math
import os
import struct
import sys
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.msg import (HandCommand, HandCommandArray, HandType, JointCommand,
                            JointCommandArray, JointStateArray, MessageHeader,
                            UpperBodyCommandArray)
from sensor_msgs.msg import PointCloud2

_IK_SDK_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "ik_sdk")
if _IK_SDK_PATH not in sys.path:
    sys.path.insert(0, _IK_SDK_PATH)

ARM_TOPIC = "/aima/hal/joint/arm/command"
UPPER_BODY_TOPIC = "/mc/upper_body_command"
ARM_STATE_TOPIC = "/aima/hal/joint/arm/state"
HAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_ARM = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_yaw_joint",
            "left_wrist_pitch_joint", "left_wrist_roll_joint"]
RIGHT_ARM = [name.replace("left_", "right_") for name in LEFT_ARM]
ALL_ARM_JOINTS = LEFT_ARM + RIGHT_ARM
# The generated MuJoCo fingers are centred 13 cm along the official
# OmniPicker base frame's local +Z axis.  IK must transform this as a tool
# point; it is not a fixed offset in pelvis/world axes.
CLAW_TOOL_OFFSET = (0.0, 0.0, 0.13)


def compose_arm_target(current: Iterable[float], side: str,
                       active: Iterable[float]) -> list[float]:
    """只替换目标侧七关节，另一侧保持当前角度。"""
    result, values = list(current), list(active)
    if len(result) != 14 or len(values) != 7:
        raise ValueError("current 必须为14维，active 必须为7维")
    start = 0 if side == "left" else 7
    result[start:start + 7] = values
    return result


def world_to_base(world_xyz: Iterable[float], base_xy_yaw: Iterable[float]) -> list[float]:
    """把 map/world 点转换到 IK 的 pelvis 坐标系。

    新调用传入 ``(x, y, z, yaw)``。为兼容旧代码，三元组仍按
    ``(x, y, yaw)`` 处理，但其 z 原点会被视为世界地面。
    """
    x, y, z = world_xyz
    base = list(base_xy_yaw)
    if len(base) == 4:
        bx, by, bz, yaw = base
    elif len(base) == 3:
        bx, by, yaw = base
        bz = 0.0
    else:
        raise ValueError("base pose 必须是 (x,y,yaw) 或 (x,y,z,yaw)")
    dx, dy = x - bx, y - by
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * dx + s * dy, -s * dx + c * dy, z - bz]


class GraspController:
    def __init__(self, node: Node, sim: bool = True):
        self._node, self._sim, self._ik_solver = node, sim, None
        self._arm_positions = None
        self._held_arm_target = None
        self._upper_sequence = 0
        self._object_positions = {}
        self._claw_positions = {}
        self._held_object_name = None
        self._held_object_initial_z = None
        self._arm_pub = node.create_publisher(
            UpperBodyCommandArray, UPPER_BODY_TOPIC, 10)
        self._sim_claw_pub = node.create_publisher(
            JointCommandArray, ARM_TOPIC, 10)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            JointStateArray, ARM_STATE_TOPIC, self._on_arm_state, state_qos)
        node.create_subscription(
            PointCloud2, "/aima/sim/lidar/points", self._on_object_cloud,
            state_qos)
        hand_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._hand_pub = node.create_publisher(HandCommandArray, HAND_TOPIC, hand_qos)

    def _on_arm_state(self, msg: JointStateArray):
        positions = {joint.name: joint.position for joint in msg.joints}
        if all(name in positions for name in ALL_ARM_JOINTS):
            self._arm_positions = [positions[name] for name in ALL_ARM_JOINTS]

    def _on_object_cloud(self, msg: PointCloud2):
        if msg.width * msg.height < 5:
            return
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(axis in offsets for axis in "xyz"):
            return
        endian = ">f" if msg.is_bigendian else "<f"
        for index, name in enumerate(("药盒", "水杯", "面包"), start=2):
            base = index * msg.point_step
            point = tuple(struct.unpack_from(
                endian, msg.data, base + offsets[axis])[0] for axis in "xyz")
            if all(math.isfinite(value) for value in point):
                self._object_positions[name] = point
        if msg.width * msg.height >= 7:
            for index, side in ((5, "left"), (6, "right")):
                base = index * msg.point_step
                point = tuple(struct.unpack_from(
                    endian, msg.data, base + offsets[axis])[0] for axis in "xyz")
                if all(math.isfinite(value) for value in point):
                    self._claw_positions[side] = point

    @property
    def ik(self):
        if self._ik_solver is None:
            try:
                from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
                self._ik_solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
                self._node.get_logger().info("OmniPicker IK Solver 已加载")
            except Exception as exc:
                self._node.get_logger().error(f"IK SDK 不可用: {exc}")
        return self._ik_solver

    def object_position(self, name: str):
        """Latest physical world position reported by the simulation sensor."""
        rclpy.spin_once(self._node, timeout_sec=0.05)
        return self._object_positions.get(name)

    def wait_for_arm_state(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while self._arm_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return self._arm_positions is not None

    def solve_ik(self, side: str, target_xyz: Iterable[float],
                 tool_offset_xyz: Iterable[float] | None = None) -> list[float] | None:
        if side not in ("left", "right"):
            raise ValueError(side)
        if self.ik is None or not self.wait_for_arm_state():
            self._node.get_logger().error("无 IK 或手臂状态，拒绝虚假抓取")
            return None
        from x2_ik_sdk import ArmSide
        result = self.ik.solve_position(
            ArmSide.LEFT if side == "left" else ArmSide.RIGHT,
            list(target_xyz), self._arm_positions,
            tool_offset_xyz=tool_offset_xyz)
        if not result.success:
            self._node.get_logger().error(
                f"IK 未收敛 error={result.error_norm:.4f}: {result.message}")
            return None
        return result.active_arm

    def move_arm(self, target: Iterable[float], duration: float = 1.8) -> bool:
        target = list(target)
        if len(target) != 14 or not self.wait_for_arm_state():
            return False
        start = list(self._arm_positions)
        steps, period = max(2, round(duration * 50)), 0.02
        for step in range(1, steps + 1):
            alpha = 0.5 - 0.5 * math.cos(math.pi * step / steps)
            values = [a + (b - a) * alpha for a, b in zip(start, target)]
            self._publish_upper_body(values)
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(period)
        self._held_arm_target = list(target)
        # The closed-source upper-body controller follows the 50 Hz command
        # with visible lag. Keep the final packet alive long enough for the
        # physical arm to settle instead of starting the next phase while it
        # is still near the previous waypoint.
        self._hold_arm_pose(0.6)
        return True

    def _hold_arm_pose(self, duration: float) -> bool:
        if self._held_arm_target is None:
            return False
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._publish_upper_body(self._held_arm_target)
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(0.02)
        return True

    def _publish_upper_body(self, arm_values: Iterable[float]) -> None:
        """发 /mc/upper_body_command → MC 转发到手臂/sim。"""
        arm_list = [float(v) for v in arm_values]
        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self._upper_sequence
        self._upper_sequence += 1
        msg.source = "remote_teleop_pc"
        msg.hand_sub_mode = UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE
        msg.head_pos = [0.0, 0.0]
        msg.arm_pos = arm_list
        msg.hand_pos = [1.0, 1.0]
        self._arm_pub.publish(msg)

    def grip(self, hand: str, position: float, duration: float = 1.5):
        if hand not in ("left", "right"):
            raise ValueError(hand)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            msg = HandCommandArray()
            msg.header = MessageHeader()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            cmd = HandCommand()
            cmd.name = f"{hand}_claw_joint"
            cmd.position, cmd.velocity = float(position), 0.3
            cmd.acceleration, cmd.deceleration, cmd.effort = 0.5, 0.5, 8.0
            if hand == "left":
                msg.left_hand_type = HandType(value=0x2)
                msg.right_hand_type = HandType(value=0x0)
                msg.left_hands.append(cmd)
            else:
                msg.right_hand_type = HandType(value=0x2)
                msg.left_hand_type = HandType(value=0x0)
                msg.right_hands.append(cmd)
            self._hand_pub.publish(msg)
            # upper_body_external expires its source after 1 s.  Keep the
            # lifted arm posture alive while closing and during announcement.
            if self._held_arm_target is not None:
                self._publish_upper_body(self._held_arm_target)
            # MuJoCo 覆盖模型把物理滑轨加入 arm 闭环；真实机器忽略此内部关节名。
            # HandCommandArray 仍始终发布，因此真机接口保持官方规定格式。
            if self._sim:
                physical = JointCommandArray()
                physical.header = MessageHeader()
                physical.header.stamp = self._node.get_clock().now().to_msg()
                claw = JointCommand()
                claw.name = "L_claw_joint" if hand == "left" else "R_claw_joint"
                claw.position = 0.04 if position > 0.5 else 0.0
                claw.velocity, claw.stiffness, claw.damping = 0.15, 100.0, 4.0
                physical.joints.append(claw)
                self._sim_claw_pub.publish(physical)
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(0.02)

    def _move_active_arm(self, side: str, target_xyz: Iterable[float],
                         tool_offset_xyz: Iterable[float] | None = None) -> bool:
        active = self.solve_ik(side, target_xyz, tool_offset_xyz)
        if active is None:
            return False
        return self.move_arm(compose_arm_target(self._arm_positions, side, active))

    def grasp_and_lift(self, side: str, target_base_xyz: Iterable[float],
                       lift_height: float = 0.15,
                       object_name: str | None = None) -> bool:
        """预抓取→接近→闭爪→垂直抬升；成功后保持闭爪和抬升姿态。"""
        target = list(target_base_xyz)
        initial_object = self._object_positions.get(object_name)
        if self._sim and object_name and initial_object is None:
            self._node.get_logger().error(f"没有{object_name}的物理位姿反馈")
            return False
        pregrasp = [target[0] - 0.10, target[1], target[2] + 0.03]
        lifted = [target[0], target[1], target[2] + lift_height]
        self.grip(side, 1.0)
        if not self._move_active_arm(side, pregrasp, CLAW_TOOL_OFFSET):
            return False
        if not self._move_active_arm(side, target, CLAW_TOOL_OFFSET):
            return False
        # Closing while the arm was still advancing from the pregrasp pose
        # pushed the mug sideways and produced a false 2 mm "lift". Hold the
        # contact pose first; only then inspect geometry and close the fingers.
        self._hold_arm_pose(2.0)
        if self._sim and object_name:
            claw = self._claw_positions.get(side)
            obj = self._object_positions.get(object_name)
            if claw is not None and obj is not None:
                delta = tuple(claw[i] - obj[i] for i in range(3))
                self._node.get_logger().info(
                    f"闭爪前物理坐标: {side}_claw={claw}, "
                    f"{object_name}={obj}, claw-object={delta}")
        self.grip(side, 0.0, duration=2.0)
        if not self._move_active_arm(side, lifted, CLAW_TOOL_OFFSET):
            return False
        # 再次施加闭合命令，随后调用者须在播报期间调用 hold_grip。
        self.grip(side, 0.0, duration=0.5)
        if self._sim and object_name:
            current = self._object_positions.get(object_name)
            if current is None or current[2] < initial_object[2] + 0.08:
                actual = "无反馈" if current is None else f"{current[2] - initial_object[2]:.3f}m"
                self._node.get_logger().error(
                    f"{object_name}未真实离桌 8cm，实测抬升={actual}")
                return False
            self._held_object_name = object_name
            self._held_object_initial_z = initial_object[2]
            self._node.get_logger().info(
                f"物理抓取确认: {object_name}抬升"
                f"{current[2] - initial_object[2]:.3f}m")
        return True

    def hold_grip(self, side: str, duration: float) -> bool:
        self.grip(side, 0.0, duration=duration)
        if self._sim and self._held_object_name:
            current = self._object_positions.get(self._held_object_name)
            if (current is None or self._held_object_initial_z is None or
                    current[2] < self._held_object_initial_z + 0.06):
                self._node.get_logger().error("播报期间物品掉落")
                return False
        return True

    def grasp_object(self, side: str, target_xyz: list, place_xyz: list = None):
        """兼容旧入口；target_xyz 必须已是 base 坐标。"""
        if place_xyz is not None:
            self._node.get_logger().warn("国赛流程不应在播报前放下物品，忽略 place_xyz")
        return self.grasp_and_lift(side, target_xyz)
