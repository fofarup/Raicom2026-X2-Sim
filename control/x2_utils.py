#!/usr/bin/env python3
"""X2 智慧养老任务公共工具模块。

提供模式切换、输入源注册、速度控制+航向修正、导航、表情、TTS、
手臂/手部控制等公共能力。仿真模式下用 print 模拟语音和表情。
"""

import math
import time
from typing import Optional, Tuple, Callable

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from aimdk_msgs.msg import (
    McLocomotionVelocity,
    RequestHeader,
    MessageHeader,
    JointCommandArray,
    JointCommand,
    HandCommandArray,
    HandCommand,
)
from aimdk_msgs.srv import (
    SetMcAction,
    SetMcInputSource,
    GetAllJointState,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── 仿真配置 ──────────────────────────────────────────────────
class SimConfig:
    """比赛场地坐标配置（4m×4m 地图）。可根据实际场地修改。"""
    # 各区域目标点
    START_ZONE     = (-1.5, -1.5)   # 出发区
    INTERACT_I     = (0.0, 1.0)     # 交互区-I
    INTERACT_II    = (0.5, 1.0)     # 交互区-II
    WORK_ZONE      = (1.5, -1.4)    # 作业区（桌子位置）

    # 物品坐标（作业区桌面）
    OBJECTS = {
        "药盒":   (1.3, -1.2, 0.65),
        "杯子":   (1.5, -1.3, 0.65),
        "面包":   (1.7, -1.2, 0.65),
    }

    # 机器人起点
    ROBOT_START = (-1.5, -1.5)

    # 控制参数
    DEFAULT_SPEED        = 0.15   # m/s
    MAX_SPEED            = 0.50   # m/s
    HEADING_KP           = 0.8    # 航向修正 P
    POSITION_TOLERANCE   = 0.15   # m，到达判定
    WAYPOINT_SPACING     = 0.3    # m，途经点间距

    # 机器人身体朝向（初始 yaw，弧度）
    INITIAL_YAW = 1.57  # 面朝 +y 方向


# ── 模式切换 ──────────────────────────────────────────────────
class ModeSwitch:
    """机器人运动模式切换。"""

    MODES = {
        "PD": "PASSIVE_DEFAULT",
        "DD": "DAMPING_DEFAULT",
        "JD": "JOINT_DEFAULT",
        "SD": "STAND_DEFAULT",
        "LD": "LOCOMOTION_DEFAULT",
    }

    def __init__(self, node: Node):
        self._node = node
        self._client = node.create_client(
            SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction"
        )

    def set(self, mode: str) -> bool:
        """切换模式（JD/SD/LD/PD/DD），等待服务就绪。"""
        if mode not in self.MODES:
            self._node.get_logger().error(f"未知模式: {mode}")
            return False
        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("SetMcAction 服务不可用")
            return False

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "raicom_task"
        cmd = req.command
        cmd.action_desc = self.MODES[mode]

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        ok = future.done() and future.result() is not None
        if ok:
            self._node.get_logger().info(f"模式已切换: {mode} ({self.MODES[mode]})")
        else:
            self._node.get_logger().error(f"模式切换失败: {mode}")
        return ok


# ── MC 输入源注册 ──────────────────────────────────────────────
class InputSource:
    """注册/注销 MC 二开输入源。"""

    SERVICE = "/aimdk_5Fmsgs/srv/SetMcInputSource"

    def __init__(self, node: Node, name: str = "raicom_task", priority: int = 40):
        self._node = node
        self._name = name
        self._priority = priority
        self._client = node.create_client(SetMcInputSource, self.SERVICE)

    def register(self) -> bool:
        if not self._client.wait_for_service(timeout_sec=10.0):
            self._node.get_logger().error("SetMcInputSource 服务不可用")
            return False
        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self._name
        req.input_source.priority = self._priority
        req.input_source.timeout = 1000
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
        ok = future.done() and future.result() is not None
        if ok:
            self._node.get_logger().info(f"MC 输入源注册: {self._name} pri={self._priority}")
        return ok


# ── 速度控制 + 航向修正 ──────────────────────────────────────
class MotionController:
    """基于里程计反馈的速度控制，带航向修正保持直线。"""

    TOPIC = "/aima/mc/locomotion/velocity"

    def __init__(self, node: Node, source_name: str = "raicom_task"):
        self._node = node
        self._source = source_name
        self._pub = node.create_publisher(McLocomotionVelocity, self.TOPIC, 10)
        self._odom = None  # 最新里程计数据

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        """获取当前位置 (x, y, z)。"""
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        return (p.x, p.y, p.z)

    @property
    def yaw(self) -> Optional[float]:
        """获取当前偏航角（弧度）。"""
        if self._odom is None:
            return None
        q = self._odom.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def on_odom(self, msg: Odometry):
        self._odom = msg

    def publish(self, forward: float, lateral: float = 0.0, angular: float = 0.0):
        msg = McLocomotionVelocity()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.source = self._source
        msg.forward_velocity = forward
        msg.lateral_velocity = lateral
        msg.angular_velocity = angular
        self._pub.publish(msg)

    def stop(self, duration: float = 1.0):
        """连续发送零速度。"""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish(0.0)
            rclpy.spin_once(self._node, timeout_sec=0.0)
            time.sleep(0.02)

    def move_toward(
        self,
        target_x: float,
        target_y: float,
        speed: float = SimConfig.DEFAULT_SPEED,
        tolerance: float = SimConfig.POSITION_TOLERANCE,
        timeout: float = 30.0,
        heading_kp: float = SimConfig.HEADING_KP,
    ) -> bool:
        """移动到目标点 (x, y)，保持航向修正走直线。

        Returns:
            True 如果到达目标点，False 如果超时。
        """
        self._node.get_logger().info(
            f"移动至 ({target_x:.2f}, {target_y:.2f}) 速度={speed:.2f}"
        )
        deadline = time.monotonic() + timeout
        spin_count = 0

        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            spin_count += 1

            if self.position is None:
                time.sleep(0.01)
                continue

            px, py, _ = self.position
            dist = math.hypot(target_x - px, target_y - py)

            if dist < tolerance:
                self._node.get_logger().info("已到达目标点")
                self.stop(1.0)
                return True

            # 计算目标航向
            target_yaw = math.atan2(target_y - py, target_x - px)
            current_yaw = self.yaw if self.yaw is not None else target_yaw

            # 航向误差 → 角速度修正
            yaw_err = target_yaw - current_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            angular = yaw_err * heading_kp
            angular = max(-0.5, min(0.5, angular))

            # 速度（近目标减速）
            fwd = speed * min(1.0, dist / 0.5)

            if spin_count % 20 == 0:
                self._node.get_logger().info(
                    f"  pos=({px:.2f},{py:.2f}) "
                    f"dist={dist:.2f} yaw_err={math.degrees(yaw_err):.1f}°"
                )

            self.publish(fwd, 0.0, angular)
            time.sleep(0.02)

        self._node.get_logger().warn("移动超时！")
        self.stop(1.0)
        return False


# ── 途经点导航 ────────────────────────────────────────────────
class WaypointNavigator:
    """依次途经多点，到达最终目标。"""

    def __init__(self, motion: MotionController):
        self._mc = motion

    def go(
        self,
        waypoints: list,
        speed: float = SimConfig.DEFAULT_SPEED,
        timeout_per_point: float = 30.0,
    ) -> bool:
        """依次到达途经点列表 [(x, y), ...]。

        Returns:
            True 全部到达，False 中途失败。
        """
        for i, (x, y) in enumerate(waypoints):
            self._mc._node.get_logger().info(f"--- 途经点 {i+1}/{len(waypoints)} ---")
            if not self._mc.move_toward(x, y, speed=speed, timeout=timeout_per_point):
                return False
        return True


# ── 表情控制 ──────────────────────────────────────────────────
class ExpressionController:
    """控制 X2 面部表情。仿真模式下用 print 模拟，
    真机通过 /aimdk_5Fmsgs/srv/ 或屏幕服务控制。"""

    EXPRESSIONS = ["快乐", "悲伤", "愤怒", "睡觉", "充电", "疑惑", "平静-卖萌", "平静"]

    def __init__(self, node: Node, sim_mode: bool = True):
        self._node = node
        self._sim = sim_mode

    def show(self, expression: str):
        if self._sim:
            self._node.get_logger().info(f"[表情] 😀 {expression}")
        else:
            # TODO: 真机屏幕 expression 控制 API
            self._node.get_logger().info(f"设置表情: {expression}")


# ── 语音 / TTS ────────────────────────────────────────────────
class SpeechController:
    """机器人语音输出。仿真模式用 print。"""

    def __init__(self, node: Node, sim_mode: bool = True):
        self._node = node
        self._sim = sim_mode

    def say(self, text: str):
        if self._sim:
            self._node.get_logger().info(f"[TTS] 🔈 {text}")
        else:
            # TODO: 真机 TTS API
            self._node.get_logger().info(f"TTS: {text}")

    def listen(self, prompt: str = "") -> str:
        """获取语音输入。仿真模式用键盘输入。"""
        if self._sim:
            if prompt:
                print(f"\n🤖 {prompt}")
            return input("🎤 请输入（模拟语音）: ").strip()
        else:
            # TODO: 真机 ASR API
            return input("语音输入（真机将调用 ASR）: ").strip()


# ── 手臂控制 ──────────────────────────────────────────────────
class ArmController:
    """手臂关节控制。"""

    TOPIC_ARM = "/aima/hal/joint/arm/command"
    # 左臂 7 关节 + 右臂 7 关节
    JOINT_NAMES = [
        "left_shoulder_pitch", "left_shoulder_pitch", "left_shoulder_roll",
        "left_shoulder_yaw", "left_elbow", "left_wrist_yaw",
        "left_wrist_pitch", "left_wrist_roll",
        "right_shoulder_pitch", "right_shoulder_pitch", "right_shoulder_roll",
        "right_shoulder_yaw", "right_elbow", "right_wrist_yaw",
        "right_wrist_pitch", "right_wrist_roll",
    ]

    # 实际手臂关节顺序（参照关节控制文档）
    ARM_LEFT = [
        "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
        "left_elbow", "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
    ]
    ARM_RIGHT = [
        "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
        "right_elbow", "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
    ]

    def __init__(self, node: Node, sim_mode: bool = True):
        self._node = node
        self._sim = sim_mode
        self._pub = node.create_publisher(JointCommandArray, self.TOPIC_ARM, 10)

    def _make_cmd(self, joint_names: list, positions: list) -> JointCommandArray:
        arr = JointCommandArray()
        arr.header = MessageHeader()
        arr.header.stamp = self._node.get_clock().now().to_msg()
        for name, pos in zip(joint_names, positions):
            cmd = JointCommand()
            cmd.name = name
            cmd.position = float(pos)
            cmd.velocity = 0.1
            cmd.effort = 0.0
            cmd.stiffness = 0.0
            cmd.damping = 0.0
            arr.joints.append(cmd)
        return arr

    def goto(
        self,
        left_positions: list,
        right_positions: list,
        steps: int = 50,
        interval: float = 0.02,
    ):
        """插值移动到目标手臂姿态（仿真用 print 模拟关节运动）。"""
        if self._sim:
            self._node.get_logger().info(
                f"[手臂] 左臂→{left_positions[:3]}... 右臂→{right_positions[:3]}..."
            )
            time.sleep(1.0)
            return

        all_joints = self.ARM_LEFT + self.ARM_RIGHT
        all_targets = list(left_positions) + list(right_positions)

        for step in range(steps):
            t = (step + 1) / steps
            interp = [t * v for v in all_targets]
            msg = self._make_cmd(all_joints, interp)
            self._pub.publish(msg)
            time.sleep(interval)


# ── 手部 / 夹爪控制 ───────────────────────────────────────────
class HandController:
    """末端执行器（夹爪/灵巧手）控制。"""

    TOPIC_HAND = "/aima/hal/joint/hand/command"

    def __init__(self, node: Node, sim_mode: bool = True):
        self._node = node
        self._sim = sim_mode
        self._pub = node.create_publisher(HandCommandArray, self.TOPIC_HAND, 10)

    def grip(self, position: float = 0.0):
        """夹爪控制。0.0=全开，1.0=全闭。"""
        if self._sim:
            state = "闭合 ✊" if position > 0.5 else "张开 ✋"
            self._node.get_logger().info(f"[手部] 夹爪 {state} (pos={position})")
            return

        msg = HandCommandArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        cmd = HandCommand()
        cmd.name = "gripper"
        cmd.position = float(position)
        cmd.velocity = 0.3
        cmd.effort = 0.0
        msg.left_hands.append(cmd)
        msg.right_hands.append(cmd)
        self._pub.publish(msg)


# ── 里程计节点 ────────────────────────────────────────────────
class OdomNode(Node):
    """独立 ROS 节点，订阅里程计并对外暴露位置信息。"""

    ODOM_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    def __init__(self, name: str = "x2_task_node"):
        super().__init__(name)
        self.motion = MotionController(self, name)
        self._odom_sub = self.create_subscription(
            Odometry, "/aima/hal/odom/state", self._odom_cb, self.ODOM_QOS
        )

    def _odom_cb(self, msg: Odometry):
        self.motion.on_odom(msg)

    @property
    def x(self) -> float:
        return self.motion.position[0] if self.motion.position else 0.0

    @property
    def y(self) -> float:
        return self.motion.position[1] if self.motion.position else 0.0


# ── 初始化工具 ────────────────────────────────────────────────
def init_robot(node: Node, sim: bool = True) -> dict:
    """一站式初始化：SD→LD，注册输入源。

    Returns:
        dict 含 mode, input_src, motion, expr, speech, arm, hand 控制器。
    """
    mode = ModeSwitch(node)
    src = InputSource(node, "raicom_task", 40)

    # 注：调用方应确保已手动在 MuJoCo 中 SD+Reset
    if not mode.set("JD"):
        node.get_logger().warn("JD 切换失败，可能已处于目标模式")
    rclpy.spin_once(node, timeout_sec=0.1)
    if not mode.set("SD"):
        node.get_logger().warn("SD 切换失败")
    node.get_logger().info("⚠️  请确保已在 MuJoCo 中点击 Reset！")

    return {
        "mode": mode,
        "input_src": src,
        "motion": MotionController(node),
        "expr": ExpressionController(node, sim),
        "speech": SpeechController(node, sim),
        "arm": ArmController(node, sim),
        "hand": HandController(node, sim),
    }


def set_ready(mode: ModeSwitch, input_src: InputSource) -> bool:
    """切换到 LD 模式并注册输入源。"""
    if not mode.set("LD"):
        return False
    return input_src.register()
