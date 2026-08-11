#!/usr/bin/env python3
"""ROS-independent intent routing for the full RAICOM voice workflow."""

from __future__ import annotations

from dataclasses import dataclass
import re

from task3_needs import NeedDecision, classify_need


EMOJIS = ("悲伤", "睡觉", "愤怒", "快乐", "充电")
MOTIONS = ("挥左手", "挥右手", "左手敬礼", "右手敬礼", "双手交叉", "双手打叉")


@dataclass(frozen=True)
class VoiceIntent:
    kind: str
    value: str = ""
    need: NeedDecision | None = None
    confidence: float = 1.0
    source: str = "local"
    normalized_text: str = ""


def _compact(text: str) -> str:
    value = "".join(text.lower().split())
    for mark in "，。！？,.!?；;：:":
        value = value.replace(mark, "")
    return value


# SenseVoice may substitute characters with the same/similar pronunciation.
# Keep this list deliberately narrow: every replacement must be harmless in the
# competition command vocabulary.
PHONETIC_REPLACEMENTS = (
    (r"[灰惠回会晖辉]([左右]手)", r"挥\1"),
    (r"([左右])边的手", r"\1手"),
    (r"[经精晶惊]理", "敬礼"),
    (r"经[历力励]", "敬礼"),
    (r"精力", "敬礼"),
    (r"头[藤腾]", "头疼"),
    (r"口[柯可]", "口渴"),
)


def normalize_asr_text(text: str) -> str:
    """Apply conservative ASR-error repair without changing free-form meaning."""
    value = _compact(text)
    for pattern, replacement in PHONETIC_REPLACEMENTS:
        value = re.sub(pattern, replacement, value)
    return value


def parse_voice_intent(text: str) -> VoiceIntent:
    compact = normalize_asr_text(text)
    # Safety and recovery always win over scoring-task phrases.
    if any(phrase in compact for phrase in ("停止任务", "立即停止", "马上停止", "紧急停止", "停下来", "别动")):
        return VoiceIntent("stop")
    if any(phrase in compact for phrase in ("回到出发区", "返回出发区", "回出发区", "回到起点", "返回起点")):
        return VoiceIntent("navigate", "start")
    if any(phrase in compact for phrase in ("重新去交互区", "回到交互区", "返回交互区", "去交互区", "进入交互区")):
        return VoiceIntent("navigate", "interaction")
    if any(phrase in compact for phrase in ("去作业区", "前往作业区", "到作业区")):
        return VoiceIntent("navigate", "work")
    if any(phrase in compact for phrase in ("当前在哪里", "现在在哪里", "你在哪里", "当前位置", "当前坐标")):
        return VoiceIntent("status")
    if any(phrase in compact for phrase in ("开始执行任务", "开始任务", "执行任务", "比赛开始")):
        return VoiceIntent("start_flow")

    need = classify_need(compact)
    if need is not None:
        return VoiceIntent("need", need=need)

    if any(word in compact for word in ("几点", "时间")):
        return VoiceIntent("task2", "time")
    if any(word in compact for word in (
        "数字", "颜色", "图片", "图中", "看见", "看到",
        "这是什么", "这是什", "这是啥", "这上面是什么",
    )):
        return VoiceIntent("task2", "vision")
    for name in EMOJIS:
        if name in compact:
            return VoiceIntent("task2", name, normalized_text=compact)

    side = ""
    if any(word in compact for word in ("左手", "左边", "左侧")):
        side = "左"
    elif any(word in compact for word in ("右手", "右边", "右侧")):
        side = "右"
    if any(word in compact for word in (
        "挥手", "挥一下", "打招呼", "打个招呼", "打声招呼", "招招手", "摆摆手",
    )):
        if side:
            return VoiceIntent("task2", f"挥{side}手", normalized_text=compact)
    if "敬礼" in compact or "行礼" in compact:
        if side:
            return VoiceIntent("task2", f"{side}手敬礼", normalized_text=compact)
        # The scoring set defines a side; do not guess when ASR omitted it.
        return VoiceIntent("unknown", confidence=0.35, normalized_text=compact)
    both_hands = any(word in compact for word in ("双手", "两手", "两只手"))
    cross_cue = any(word in compact for word in ("交叉", "打叉", "打个叉", "画叉", "叉一下"))
    if (both_hands and cross_cue) or any(word in compact for word in ("交叉双手",)):
        return VoiceIntent("task2", "双手交叉", normalized_text=compact)
    for name in MOTIONS:
        if name in compact:
            return VoiceIntent("task2", name, normalized_text=compact)

    return VoiceIntent("unknown", confidence=0.0, normalized_text=compact)


def run_intent_tests() -> bool:
    cases = {
        "开始执行任务": ("start_flow", ""),
        "停止任务": ("stop", ""),
        "机器人停下来": ("stop", ""),
        "回到出发区": ("navigate", "start"),
        "重新去交互区": ("navigate", "interaction"),
        "去作业区": ("navigate", "work"),
        "你当前在哪里": ("status", ""),
        "现在几点了": ("task2", "time"),
        "识别图片里的数字和颜色": ("task2", "vision"),
        "请问这是什么": ("task2", "vision"),
        "这是啥": ("task2", "vision"),
        "做一个快乐表情": ("task2", "快乐"),
        "挥左手": ("task2", "挥左手"),
        "灰右手跟大家打个招呼": ("task2", "挥右手"),
        "用左边的手招招手": ("task2", "挥左手"),
        "请用右手经理": ("task2", "右手敬礼"),
        "左手经历一下": ("task2", "左手敬礼"),
        "把两只手交叉一下": ("task2", "双手交叉"),
        "用双手给我打个叉": ("task2", "双手交叉"),
        "用左手给我打个招呼": ("task2", "挥左手"),
        "早上起来头就隐隐作痛，到现在还没缓过来。": ("need", "药盒"),
        "我有点口渴了，想喝点水。": ("need", "一次性纸杯"),
        "我现在有点饿了，想吃点东西。": ("need", "小面包"),
        "今天天气不错": ("unknown", ""),
    }
    passed = True
    for text, expected in cases.items():
        intent = parse_voice_intent(text)
        value = intent.need.item if intent.need else intent.value
        actual = (intent.kind, value)
        ok = actual == expected
        print(f"{'PASS' if ok else 'FAIL'} {text} -> {actual}")
        passed &= ok
    return passed


if __name__ == "__main__":
    raise SystemExit(0 if run_intent_tests() else 1)
