"""五种抽签动作的双臂关节轨迹——基于官方 AimDK 关节限位。

关节顺序: shoulder_pitch, shoulder_roll, shoulder_yaw,
          elbow, wrist_yaw, wrist_pitch, wrist_roll
限位来源: https://x2-aimdk.agibot.com 示例 joint_control
"""

import time
from .grasp import GraspController, ALL_ARM_JOINTS

# ── 官方关节限位（AimDK SDK joint_control 示例）──────────────
# left_shoulder: pitch[-3.08,2.04] roll[-0.061,2.993] yaw[-2.556,2.556]
# left_elbow: [-2.3556,0] wrist_yaw[-2.556,2.556] pitch[-0.558,0.558] roll[-1.571,0.724]
# right_shoulder: pitch[-3.08,2.04] roll[-2.993,0.061] yaw[-2.556,2.556]
# right_elbow: [-2.3556,0] wrist_yaw[-2.556,2.556] pitch[-0.558,0.558] roll[-0.724,1.571]

# ── 预备姿态：双臂微屈自然下垂 ────────────────────────────────
READY = [0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0,
         0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0]

# ── 五种比赛动作（全部在官方限位内）──────────────────────────
POSES = {
    # 挥左手：左臂前上方举起，手腕摆动
    "挥左手":
        [-0.70, 0.45, 0.0, -1.20, 0.0, 0.25, 0.0] + READY[7:],

    # 挥右手：右臂前上方举起
    "挥右手":
        READY[:7] + [-0.70, -0.45, 0.0, -1.20, 0.0, -0.25, 0.0],

    # 左手敬礼：左手抬至额头
    "左手敬礼":
        [-0.85, 0.30, -0.20, -1.35, 0.0, 0.30, 0.0] + READY[7:],

    # 右手敬礼：右手抬至额头
    "右手敬礼":
        READY[:7] + [-0.85, -0.30, 0.20, -1.35, 0.0, -0.30, 0.0],

    # 双手打叉：双臂斜上举，在空中形成 X 形
    "双手打叉": [
        # 左臂：前上方举起 → 外展 → 肘微屈 → 左斜上
        -1.20, 1.50, 0.0, -0.80, 0.0, 0.0, 0.0,
        # 右臂：前上方举起 → 外展 → 肘微屈 → 右斜上
        -1.20,-1.50, 0.0, -0.80, 0.0, 0.0, 0.0,
    ],
}


class GestureController:
    def __init__(self, grasp: GraspController):
        self._grasp = grasp

    def perform(self, name: str) -> bool:
        if name not in POSES:
            self._grasp._node.get_logger().error(f"未知手势: {name}")
            return False
        target = POSES[name]
        if not self._grasp.move_arm(target, duration=1.5):
            return False
        # 挥手额外摆动腕关节
        if name.startswith("挥"):
            side_start = 0 if "左" in name else 7
            for angle in (0.45, -0.45, 0.45, 0.0):
                pose = list(target)
                pose[side_start + 6] = angle  # wrist_roll
                if not self._grasp.move_arm(pose, duration=0.28):
                    return False
        else:
            time.sleep(1.5)
        return True

    def return_to_ready(self) -> bool:
        return self._grasp.move_arm(READY, duration=1.2)
