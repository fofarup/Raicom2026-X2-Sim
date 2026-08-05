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
    # 挥左手：人工调参，wrist_pitch 来回摇摆
    "挥左手":
        [-1.32, 0.87, 0.28, -1.40, -0.64, 0.00, -0.12] + READY[7:],

    # 挥右手：左手镜像
    "挥右手":
        READY[:7] + [-1.32, -0.87, -0.28, -1.40, 0.64, 0.00, 0.12],

    # 左手敬礼：人工调参保存，大臂外展+手到额头
    "左手敬礼":
        [-1.37, 1.57, 0.04, -2.20, 1.38, 0.15, 0.0] + READY[7:],

    # 右手敬礼：左手敬礼镜像
    "右手敬礼":
        READY[:7] + [-1.37, -1.57, -0.04, -2.20, -1.38, 0.15, 0.0],

    # 双手打叉：人工调参保存，肩roll到极限内收+前臂交叉
    "双手打叉": [
        # 左臂：肩前送+roll极限内收+yaw内旋+肘深屈
        -0.92, -0.061, -0.679, -1.767, 0.00, 0.15, 0.00,
        # 右臂：肩前送+roll极限内收+yaw内旋+肘屈(较浅=前)
        -1.04, 0.061, 0.660, -1.196, 0.12, -0.158, -0.114,
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
        # 挥手：手腕俯仰来回摆（wrist_pitch 振荡）
        if name.startswith("挥"):
            side_start = 0 if "左" in name else 7
            for angle in (0.0, -0.50, 0.50, -0.50, 0.50, 0.0):
                pose = list(target)
                pose[side_start + 5] = angle  # wrist_pitch
                if not self._grasp.move_arm(pose, duration=0.25):
                    return False
        else:
            time.sleep(1.5)
        return True

    def return_to_ready(self) -> bool:
        return self._grasp.move_arm(READY, duration=1.2)
