"""五种抽签动作的双臂关节轨迹。"""
import time
from .grasp import GraspController

# 关节顺序见 grasp.ALL_ARM_JOINTS；姿态幅度保守，适合站立状态。
READY = [0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0,
         0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0]
POSES = {
    "挥左手": [-0.70, 0.45, 0.0, -1.20, 0.0, 0.25, 0.0] + READY[7:],
    "挥右手": READY[:7] + [-0.70, -0.45, 0.0, -1.20, 0.0, -0.25, 0.0],
    "左手敬礼": [-0.85, 0.30, -0.20, -1.35, 0.0, 0.30, 0.0] + READY[7:],
    "右手敬礼": READY[:7] + [-0.85, -0.30, 0.20, -1.35, 0.0, -0.30, 0.0],
    "双手打叉": [-0.25, 0.55, 0.30, -1.30, 0.0, 0.0, 0.0,
                 -0.25, -0.55, -0.30, -1.30, 0.0, 0.0, 0.0],
}


class GestureController:
    def __init__(self, grasp: GraspController):
        self._grasp = grasp

    def perform(self, name: str) -> bool:
        if name not in POSES:
            raise ValueError(name)
        if not self._grasp.move_arm(POSES[name], duration=1.5):
            return False
        # 挥手额外摆动腕关节，其他动作保持足够展示时间。
        if name.startswith("挥"):
            side_start = 0 if "左" in name else 7
            for angle in (0.45, -0.45, 0.45, 0.0):
                pose = list(POSES[name])
                pose[side_start + 6] = angle
                if not self._grasp.move_arm(pose, duration=0.28):
                    return False
        else:
            time.sleep(1.5)
        return True

    def return_to_ready(self) -> bool:
        """动作展示后回到安全行走/抓取准备姿态。"""
        return self._grasp.move_arm(READY, duration=1.2)
