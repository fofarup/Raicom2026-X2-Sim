"""比赛输入、意图和状态定义；不依赖 ROS，便于穷举测试。"""
from dataclasses import dataclass
from enum import Enum
import re


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

# ---- 导航意图关键词 ----
NAV_KEYWORDS = ("前往交互区", "去交互区", "开始导航", "开始任务",
                "前往交互区一", "前往交互区1", "导航到交互区")

# ---- 时间意图关键词 ----
TIME_KEYWORDS = ("几点了", "几点", "现在几点", "时间", "现在时间",
                 "报时", "告诉我时间", "什么时间", "当前时间")

# ---- 确认/否认关键词 ----
CONFIRM_KEYWORDS = ("对", "是的", "确认", "正确", "没错", "好的",
                    "可以", "行", "嗯", "对的", "是")
DENY_KEYWORDS = ("不对", "不是", "错了", "不对的", "不", "错", "重新")


def is_nav_command(text: str) -> bool:
    """判断语音输入是否为导航指令。"""
    return any(kw in text for kw in NAV_KEYWORDS)


def is_time_question(text: str) -> bool:
    """判断语音输入是否在问时间。"""
    return any(kw in text for kw in TIME_KEYWORDS)


def is_confirmed(text: str) -> bool:
    """判断语音输入是否为确认。"""
    return any(kw in text for kw in CONFIRM_KEYWORDS)


def is_denied(text: str) -> bool:
    """判断语音输入是否为否认。"""
    return any(kw in text for kw in DENY_KEYWORDS)


# ---- 数字/颜色语音解析 ----

# 中文数字 → int
CN_DIGIT = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10,
             "〇": 0}
CN_COLORS = [
    "粉色", "青色", "绿色", "黄色", "紫色", "深橙色", "蓝绿色",
    "蓝色", "浅蓝色", "红色", "深紫色", "靛蓝色", "黄绿色", "橙色", "浅绿色",
]


def parse_number_color(text: str) -> tuple[int | None, str | None]:
    """从语音文本中提取数字和颜色。如 '数字是5颜色红色' → (5, '红色')。"""
    digit = None
    color = None
    # 数字：先匹配阿拉伯数字，再匹配中文数字
    m = re.search(r'(\d)', text)
    if m:
        digit = int(m.group(1))
    else:
        for cn, val in CN_DIGIT.items():
            if cn in text:
                digit = val
                break
    # 颜色：找最长匹配
    for c in sorted(CN_COLORS, key=len, reverse=True):
        if c in text:
            color = c
            break
    return digit, color


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
