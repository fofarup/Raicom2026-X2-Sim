#!/usr/bin/env python3
"""Small Volcengine Ark vision client for RAICOM Task2."""

from __future__ import annotations

import base64
import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import cv2


ALLOWED_COLORS = {"红色", "橙色", "黄色", "绿色", "青色", "蓝色", "紫色", "黑色"}
COLOR_ALIASES = {
    "红": "红色", "橙": "橙色", "黄": "黄色", "绿": "绿色",
    "青": "青色", "蓝": "蓝色", "紫": "紫色", "黑": "黑色",
    "粉色": "红色", "粉红色": "红色",
}


@dataclass(frozen=True)
class VisionResult:
    ok: bool
    digit: str | None = None
    color: str | None = None
    confidence: float = 0.0
    error: str = ""


class DoubaoVisionRecognizer:
    def __init__(self, config: dict) -> None:
        self.base_url = config.get(
            "base_url", "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
        )
        self.model = str(config.get("model", "")).strip()
        self.timeout = float(config.get("timeout_seconds", 12.0))
        self.max_image_width = int(config.get("max_image_width", 960))
        self.jpeg_quality = int(config.get("jpeg_quality", 82))
        self.min_confidence = float(config.get("min_confidence", 0.55))
        self.api_key = os.environ.get("ARK_API_KEY", "").strip()
        key_file = str(config.get("api_key_file", "")).strip()
        if not self.api_key and key_file:
            try:
                self.api_key = Path(key_file).expanduser().read_text(encoding="utf-8").strip()
            except OSError:
                pass

    @property
    def configured(self) -> bool:
        return bool(self.api_key and self.model)

    def _image_data_url(self, image) -> str:
        height, width = image.shape[:2]
        if width > self.max_image_width:
            scale = self.max_image_width / width
            image = cv2.resize(
                image, (self.max_image_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise ValueError("相机画面 JPEG 编码失败")
        return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")

    @staticmethod
    def _parse_content(content: str) -> VisionResult:
        match = re.search(r"\{.*?\}", content, flags=re.S)
        if not match:
            return VisionResult(False, error="模型未返回 JSON")
        try:
            value = json.loads(match.group(0))
        except json.JSONDecodeError:
            return VisionResult(False, error="模型 JSON 格式错误")
        digit = value.get("digit")
        color = value.get("color")
        try:
            confidence = float(value.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        if digit is not None:
            digit = str(digit).strip()
        if color is not None:
            color = COLOR_ALIASES.get(str(color).strip(), str(color).strip())
        if digit not in set("0123456789") or color not in ALLOWED_COLORS:
            return VisionResult(False, error="画面中未发现有效的彩色数字卡片")
        return VisionResult(True, digit, color, max(0.0, min(1.0, confidence)))

    def recognize(self, image) -> VisionResult:
        if not self.api_key:
            return VisionResult(False, error="未配置豆包 API Key")
        if not self.model:
            return VisionResult(False, error="未配置豆包视觉模型或接入点 ID")
        prompt = (
            "这是已经校正为人眼正向、绝对不要再次旋转的机器人正前方相机画面。"
            "只识别用户手持白色卡片上最显眼的一个彩色阿拉伯数字，"
            "忽略场地广告、文字、标志、背景颜色和机器人本体。数字只允许0到9，颜色只允许"
            "红色、橙色、黄色、绿色、青色、蓝色、紫色、黑色。看不清或没有卡片时三个字段"
            "分别返回null、null、0。只输出一行JSON，不要解释："
            '{"digit":"6","color":"紫色","confidence":0.98}'
        )
        payload = {
            "model": self.model,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": self._image_data_url(image)}},
                    {"type": "text", "text": prompt},
                ],
            }],
            "temperature": 0,
            "max_tokens": 100,
        }
        request = urllib.request.Request(
            self.base_url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                body = json.loads(response.read().decode("utf-8"))
            content = body["choices"][0]["message"]["content"]
            result = self._parse_content(content)
            if result.ok and result.confidence < self.min_confidence:
                return VisionResult(False, error=f"模型置信度较低({result.confidence:.2f})")
            return result
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")[:300]
            return VisionResult(False, error=f"豆包 HTTP {exc.code}: {detail}")
        except (urllib.error.URLError, TimeoutError) as exc:
            return VisionResult(False, error=f"豆包网络请求失败: {exc}")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            return VisionResult(False, error=f"豆包响应解析失败: {exc}")
