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

# ── IK 算出的预备姿态 ──────────────────────────────────────
READY = [-0.35, 0.45, 0.0, -1.00, 0.0, 0.15, 0.0,
         -0.35,-0.45, 0.0, -1.00, 0.0, 0.15, 0.0]

# ── 五种比赛动作（IK 求解，全部在限位内）─────────────────────
POSES = {
    # 挥左手：hand → (0.25, 0.45, 0.55) 前上方
    "挥左手":
        [-0.75, 1.00, 0.25, -1.26, 0.00, 0.11, 0.04] + READY[7:],

    # 挥右手：hand → (0.25, -0.45, 0.55) 前上方
    "挥右手":
        READY[:7] + [-0.77, -1.00, -0.24, -1.26, 0.00, 0.11, -0.04],

    # 左手敬礼：人工调参保存，大臂外展+手到额头
    "左手敬礼":
        [-1.37, 1.57, 0.04, -2.20, 1.38, 0.15, 0.0] + READY[7:],

    # 右手敬礼：左手敬礼镜像
    "右手敬礼":
        READY[:7] + [-1.37, -1.57, -0.04, -2.20, -1.38, 0.15, 0.0],

    # 双手打叉：左臂低位(y=0.25,z=0.30) + 右臂高位(y=-0.15,z=0.55)
    # 一前一后(一低一高)的深度差交叉
    "双手打叉": [
        # 左臂前/低/近 → hand (0.18, 0.25, 0.30)
         0.41, 0.48, 0.06, -2.28, 0.00, -0.09, 0.01,
        # 右臂后/高/远 → hand (0.28,-0.15, 0.55)
        -1.12,-0.25, 0.21, -1.75, 0.00, 0.03, 0.03,
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
        # 挥手：前臂左右摆动（wrist_yaw 振荡）
        if name.startswith("挥"):
            side_start = 0 if "左" in name else 7
            for angle in (0.0, -0.80, 0.80, -0.80, 0.80, 0.0):
                pose = list(target)
                pose[side_start + 4] = angle  # wrist_yaw: 前臂左右
                if not self._grasp.move_arm(pose, duration=0.25):
                    return False
        else:
            time.sleep(1.5)
        return True

    def return_to_ready(self) -> bool:
        return self._grasp.move_arm(READY, duration=1.2)
