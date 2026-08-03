"""比赛输入、意图和状态定义；不依赖 ROS，便于穷举测试。"""
from dataclasses import dataclass
from enum import Enum


class CompetitionState(str, Enum):
    WAIT_START = "wait_start"
    PREPARE = "prepare"
    NAVIGATE_INTERACTION_I = "navigate_interaction_i"
    FACE_INTERACTION_II = "face_interaction_ii"
    BASIC_INTERACTION = "basic_interaction"
    UNDERSTAND_NEED = "understand_need"
    NAVIGATE_WORK_ZONE = "navigate_work_zone"
    GRASP_AND_LIFT = "grasp_and_lift"
    ANNOUNCE_WHILE_HOLDING = "announce_while_holding"
    COMPLETE = "complete"
    FAILED = "failed"


EXPRESSIONS = ("悲伤", "睡觉", "愤怒", "快乐", "充电")
GESTURES = ("挥左手", "挥右手", "左手敬礼", "右手敬礼", "双手打叉")


@dataclass(frozen=True)
class Need:
    name: str
    keywords: tuple[str, ...]
    object_name: str
    response: str
    done: str
    object_world_xyz: tuple[float, float, float]


NEEDS = (
    Need("头部不适", ("头疼", "头痛", "头部", "脑袋", "隐隐作痛", "不舒服", "难受"), "药盒",
         "听起来不太舒服，我去帮您拿药。", "已帮您拿到药盒。", (1.27, -1.60, 0.59)),
    Need("口渴", ("口渴", "喝水", "水杯"), "水杯",
         "好的，我去帮您拿杯水。", "已帮您拿到水杯。", (1.27, -1.40, 0.59)),
    Need("饥饿", ("饿", "面包", "吃的", "食物"), "面包",
         "好的，我去帮您拿点吃的。", "已帮您拿到面包。", (1.27, -1.20, 0.56)),
)


def parse_need(text: str) -> Need | None:
    normalized = "".join(text.split())
    for need in NEEDS:
        if any(keyword in normalized for keyword in need.keywords):
            return need
    return None


def validate_draw(expression: str, gesture: str, hand: str) -> None:
    if expression not in EXPRESSIONS:
        raise ValueError(f"未知表情: {expression}")
    if gesture not in GESTURES:
        raise ValueError(f"未知动作: {gesture}")
    if hand not in ("left", "right"):
        raise ValueError(f"未知夹爪: {hand}")
