#!/usr/bin/env python3
"""Optional DeepSeek fallback for ambiguous ASR transcripts.

The model never controls ROS.  Its JSON is converted to a small local allowlist.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass

from task3_needs import NeedDecision
from voice_intents import EMOJIS, MOTIONS, VoiceIntent
from deepseek_config import (
    DEEPSEEK_API_KEY,
    DEEPSEEK_BASE_URL,
    DEEPSEEK_MODEL,
    DEEPSEEK_TIMEOUT_SECONDS,
)


ALLOWED_SIMPLE = {
    "stop": ("stop", ""),
    "start_flow": ("start_flow", ""),
    "return_start": ("navigate", "start"),
    "go_interaction": ("navigate", "interaction"),
    "go_work": ("navigate", "work"),
    "status": ("status", ""),
    "tell_time": ("task2", "time"),
    "recognize_object": ("task2", "vision"),
}
ACTION_INTENTS = {
    "wave_left": "挥左手", "wave_right": "挥右手",
    "salute_left": "左手敬礼", "salute_right": "右手敬礼",
    "cross_arms": "双手交叉",
}
EMOJI_INTENTS = {
    "emoji_happy": "快乐", "emoji_sad": "悲伤",
    "emoji_angry": "愤怒", "emoji_sleep": "睡觉",
    "emoji_charge": "充电",
}
NEED_INTENTS = {
    "need_medicine": ("头痛", "药盒", "听起来不太舒服，我去帮您拿药。"),
    "need_water": ("口渴", "一次性纸杯", "好的，我去帮您拿杯水。"),
    "need_food": ("饥饿", "小面包", "好的，我去帮您拿点吃的。"),
}


@dataclass(frozen=True)
class ResolveResult:
    intent: VoiceIntent
    error: str = ""


class DeepSeekIntentResolver:
    def __init__(self) -> None:
        # Environment variables can still override the built-in private-repo
        # values during development or emergency key rotation.
        self.api_key = os.environ.get("DEEPSEEK_API_KEY", DEEPSEEK_API_KEY).strip()
        base_url = os.environ.get("DEEPSEEK_BASE_URL", DEEPSEEK_BASE_URL)
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.model = os.environ.get("DEEPSEEK_MODEL", DEEPSEEK_MODEL)
        self.timeout = float(os.environ.get("RAICOM_LLM_TIMEOUT", str(DEEPSEEK_TIMEOUT_SECONDS)))

    @property
    def enabled(self) -> bool:
        return bool(self.api_key) and os.environ.get("RAICOM_DEEPSEEK", "1") != "0"

    def resolve(self, transcript: str) -> ResolveResult:
        if not self.enabled:
            return ResolveResult(VoiceIntent("unknown", source="local"), "未配置DeepSeek")
        allowed = (
            list(ALLOWED_SIMPLE) + list(ACTION_INTENTS) +
            list(EMOJI_INTENTS) + list(NEED_INTENTS) + ["unknown"]
        )
        prompt = (
            "你是机器人比赛的ASR文本纠错和意图分类器。文本可能有中文同音错字、漏字。"
            "只能依据用户原话分类，不得补充动作。输出json对象，字段为intent、confidence、normalized_text。"
            f"intent只能是：{','.join(allowed)}。敬礼缺少左右手时返回unknown。"
            "口语如‘用左手打招呼’属于wave_left，‘用双手打个叉’属于cross_arms；"
            "头疼/脑袋痛属于need_medicine，口渴/想喝水属于need_water，"
            "饥饿/想吃东西属于need_food；询问卡片、数字或颜色属于recognize_object。"
        )
        body = json.dumps({
            "model": self.model,
            "messages": [
                {"role": "system", "content": prompt},
                {"role": "user", "content": transcript},
            ],
            "response_format": {"type": "json_object"},
            "thinking": {"type": "disabled"},
            "temperature": 0.0,
            "max_tokens": 200,
        }).encode("utf-8")
        request = urllib.request.Request(self.url, data=body, headers={
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        })
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
            content = payload["choices"][0]["message"]["content"]
            data = self._decode_json_content(content)
            return ResolveResult(self._validate(data))
        except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            return ResolveResult(VoiceIntent("unknown", source="deepseek"), type(exc).__name__)

    @staticmethod
    def _decode_json_content(content: str) -> dict:
        """Accept strict JSON plus occasional Markdown fences from model APIs."""
        value = str(content or "").strip()
        if value.startswith("```"):
            value = re.sub(r"^```(?:json)?\s*", "", value, flags=re.IGNORECASE)
            value = re.sub(r"\s*```$", "", value)
        try:
            data = json.loads(value)
        except json.JSONDecodeError:
            match = re.search(r"\{.*\}", value, flags=re.DOTALL)
            if not match:
                raise
            data = json.loads(match.group(0))
        if not isinstance(data, dict):
            raise TypeError("DeepSeek JSON root must be an object")
        return data

    @staticmethod
    def _validate(data: dict) -> VoiceIntent:
        label = str(data.get("intent", "unknown"))
        try:
            confidence = max(0.0, min(1.0, float(data.get("confidence", 0.0))))
        except (TypeError, ValueError):
            confidence = 0.0
        normalized = str(data.get("normalized_text", ""))[:200]
        # A cloud guess must be reasonably confident before physical execution.
        if confidence < 0.72:
            return VoiceIntent("unknown", confidence=confidence, source="deepseek", normalized_text=normalized)
        if label in ALLOWED_SIMPLE:
            kind, value = ALLOWED_SIMPLE[label]
            return VoiceIntent(kind, value, confidence=confidence, source="deepseek", normalized_text=normalized)
        if label in ACTION_INTENTS and ACTION_INTENTS[label] in MOTIONS:
            return VoiceIntent("task2", ACTION_INTENTS[label], confidence=confidence, source="deepseek", normalized_text=normalized)
        if label in EMOJI_INTENTS and EMOJI_INTENTS[label] in EMOJIS:
            return VoiceIntent("task2", EMOJI_INTENTS[label], confidence=confidence, source="deepseek", normalized_text=normalized)
        if label in NEED_INTENTS:
            need, item, answer = NEED_INTENTS[label]
            decision = NeedDecision(need, item, answer, confidence)
            return VoiceIntent("need", need=decision, confidence=confidence, source="deepseek", normalized_text=normalized)
        return VoiceIntent("unknown", confidence=confidence, source="deepseek", normalized_text=normalized)
