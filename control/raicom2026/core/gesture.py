"""五种抽签动作的双臂关节轨迹——用 IK 目标点驱动，不再手写关节角。"""
import math
import time
from .grasp import GraspController, ALL_ARM_JOINTS

# 预备姿态：双臂微屈自然下垂
READY = [0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0,
         0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0]

# ── 动作定义：IK 目标点 (x, y, z) 在 pelvis 坐标系 ──
# pelvis 系: +x 前, +y 左, +z 上。肩部约在 (0, ±0.25, 0.75)
# 大臂长约 0.30m，前臂长约 0.28m

GESTURE_TARGETS = {
    "挥左手":   {"side": "left",  "xyz": ( 0.25,  0.45, 1.05)},  # 左前上方
    "挥右手":   {"side": "right", "xyz": ( 0.25, -0.45, 1.05)},  # 右前上方
    "左手敬礼": {"side": "left",  "xyz": ( 0.10,  0.08, 1.15)},  # 额头左侧
    "右手敬礼": {"side": "right", "xyz": ( 0.10, -0.08, 1.15)},  # 额头右侧
    "双手打叉": {"side": "both",  # 双臂胸前交叉
                 "left_xyz":  ( 0.22, -0.12, 0.70),   # 左臂交叉到右侧
                 "right_xyz": ( 0.22,  0.12, 0.70)},  # 右臂交叉到左侧
}


class GestureController:
    def __init__(self, grasp: GraspController):
        self._grasp = grasp
        self._sim = grasp._sim

    # ── 仿真回退关节角（真机用 IK 目标点）──
    _FALLBACK = {
        "挥左手": [-0.70, 0.45, 0.0, -1.20, 0.0, 0.25, 0.0,  0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0],
        "挥右手": [ 0.20, 0.0, 0.0, -0.55, 0.0, 0.0, 0.0, -0.70,-0.45, 0.0,-1.20, 0.0,-0.25, 0.0],
        "左手敬礼":[-0.85, 0.30,-0.20,-1.35, 0.0, 0.30, 0.0,  0.20, 0.0, 0.0,-0.55, 0.0, 0.0, 0.0],
        "右手敬礼":[ 0.20, 0.0, 0.0,-0.55, 0.0, 0.0, 0.0, -0.85,-0.30, 0.20,-1.35,0.0,-0.30,0.0],
        "双手打叉":[ 0.05,-0.35, 0.60,-1.45, 0.0, 0.0, 0.0,  0.05, 0.35,-0.60,-1.45,0.0, 0.0, 0.0],
    }

    def _solve_for_gesture(self, name: str) -> list[float] | None:
        """真机用 IK，仿真用修正后的关节角。"""
        if name not in GESTURE_TARGETS:
            self._grasp._node.get_logger().error(f"未知手势: {name}")
            return None

        if self._sim:
            # 仿真模式：无手臂状态反馈，用回退关节角
            pose = self._FALLBACK.get(name)
            if pose:
                self._grasp._node.get_logger().info(f"[手势] 仿真回退 {name}")
                return pose
            return None

        # 真机模式：IK 求解
        info = GESTURE_TARGETS[name]
        if info["side"] == "both":
            left = self._grasp.solve_ik("left", info["left_xyz"])
            right = self._grasp.solve_ik("right", info["right_xyz"])
            if left is None or right is None:
                return None
            return left + right
        else:
            active = self._grasp.solve_ik(info["side"], info["xyz"])
            if active is None:
                return None
            idx = 0 if info["side"] == "left" else 7
            result = list(READY)
            result[idx:idx+7] = active
            return result

    def perform(self, name: str) -> bool:
        target = self._solve_for_gesture(name)
        if target is None:
            return False
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
