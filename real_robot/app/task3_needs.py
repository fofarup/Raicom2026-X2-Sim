#!/usr/bin/env python3
"""ROS-independent Task3 need interpretation shared by ASR and navigation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NeedDecision:
    need: str
    item: str
    answer: str
    confidence: float


NEED_RULES = (
    (
        "头痛",
        "药盒",
        "听起来不太舒服，我去帮您拿药。",
        (
            "早上起来头就隐隐作痛",
            "头就隐隐作痛",
            "头隐隐作痛",
            "还没缓过来",
            "头痛", "头疼", "脑袋疼", "头有点痛", "需要吃药", "拿药",
        ),
    ),
    (
        "口渴",
        "一次性纸杯",
        "好的，我去帮您拿杯水。",
        ("我有点口渴了想喝点水", "口渴", "口干", "想喝水", "喝点水", "拿杯水", "纸杯"),
    ),
    (
        "饥饿",
        "小面包",
        "好的，我去帮您拿点吃的。",
        ("我现在有点饿了想吃点东西", "肚子饿", "饿了", "有点饿", "想吃东西", "吃点东西", "面包"),
    ),
)


def classify_need(text: str) -> NeedDecision | None:
    """Interpret a transcript locally; never calls a cloud or robot LLM."""
    compact = "".join(text.lower().split())
    for mark in "，。！？,.!?；;：:":
        compact = compact.replace(mark, "")
    # Token-pair rules tolerate short ASR insertions/deletions such as
    # “头又隐隐作痛” while still requiring both the body/need cue and symptom.
    semantic_pairs = (
        (
            any(word in compact for word in ("头", "脑袋", "脑瓜"))
            and any(word in compact for word in ("疼", "痛", "作痛", "不舒服")),
            ("头痛", "药盒", "听起来不太舒服，我去帮您拿药。"),
        ),
        (
            any(word in compact for word in ("口", "嘴", "嗓子"))
            and any(word in compact for word in ("渴", "干", "喝水", "喝点")),
            ("口渴", "一次性纸杯", "好的，我去帮您拿杯水。"),
        ),
        (
            any(word in compact for word in ("肚子", "胃", "没吃饭"))
            and any(word in compact for word in ("饿", "空", "吃东西", "吃饭")),
            ("饥饿", "小面包", "好的，我去帮您拿点吃的。"),
        ),
    )
    for matched, (need, item, answer) in semantic_pairs:
        if matched:
            return NeedDecision(need, item, answer, 0.94)
    matches = []
    for need, item, answer, keywords in NEED_RULES:
        hit = max((len(keyword) for keyword in keywords if keyword in compact), default=0)
        if hit:
            matches.append((hit, need, item, answer))
    if not matches:
        return None
    hit, need, item, answer = max(matches, key=lambda value: value[0])
    confidence = min(0.99, 0.78 + 0.04 * hit)
    return NeedDecision(need, item, answer, confidence)


def run_demand_tests() -> bool:
    cases = {
        "早上起来头就隐隐作痛，到现在还没缓过来。": ("头痛", "药盒"),
        "我今天有点头疼": ("头痛", "药盒"),
        "早上起来头又隐隐作痛": ("头痛", "药盒"),
        "脑袋疼，帮我拿药": ("头痛", "药盒"),
        "我有点口渴了，想喝点水。": ("口渴", "一次性纸杯"),
        "麻烦拿一个纸杯": ("口渴", "一次性纸杯"),
        "我现在有点饿了，想吃点东西。": ("饥饿", "小面包"),
        "肚子饿了，想吃点东西": ("饥饿", "小面包"),
        "帮我拿小面包": ("饥饿", "小面包"),
    }
    passed = True
    for text, expected in cases.items():
        decision = classify_need(text)
        actual = None if decision is None else (decision.need, decision.item)
        ok = actual == expected
        print(f"{'PASS' if ok else 'FAIL'} {text} -> {actual}")
        passed &= ok
    unknown_ok = classify_need("今天天气不错") is None
    print(f"{'PASS' if unknown_ok else 'FAIL'} 未知需求不擅自抓取")
    return passed and unknown_ok


if __name__ == "__main__":
    raise SystemExit(0 if run_demand_tests() else 1)
