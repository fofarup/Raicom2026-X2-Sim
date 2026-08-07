"""讯飞流式语音听写（iat v2）— 云端 ASR。

比赛规则三：语音交互豁免云端算力限制，可接云端模型。
识别延迟 300-800ms（网络正常时），说话结束约 1 秒内出结果，
远快于本地 SenseVoice（CPU 2-3s）。

协议：WebSocket wss://iat-api.xfyun.cn/v2/iat
认证：RFC3986 编码 + HMAC-SHA256 签名（api_key/date/request-line）
音频：16kHz / 16bit / 单声道 PCM，分帧 1280 字节（40ms）

密钥：config/asr_keys.json（已 gitignore，不上传）
"""

import asyncio
import base64
import hashlib
import hmac
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import quote

# websockets 会自动读取代理环境变量。讯飞是国内服务必须直连：
# 全局代理（如 Clash 7897）访问讯飞会被重置。连接期间临时移除。
_PROXY_KEYS = ("http_proxy", "https_proxy", "HTTP_PROXY", "HTTPS_PROXY",
               "all_proxy", "ALL_PROXY", "no_proxy", "NO_PROXY")

HOST = "iat-api.xfyun.cn"
API_PATH = "/v2/iat"
CHUNK_BYTES = 1280  # 40ms @ 16kHz 16bit 单声道
KEYS_FILE = Path(__file__).resolve().parents[1] / "config" / "asr_keys.json"


def _rfc3986(text: str) -> str:
    """RFC 3986 百分号编码（unreserved 字符不转义）。"""
    return quote(text, safe="A-Za-z0-9-._~")


def _build_url(appid: str, api_key: str, api_secret: str) -> str:
    date = datetime.now(timezone.utc).strftime("%a, %d %b %Y %H:%M:%S GMT")
    signature_origin = f"host: {HOST}\ndate: {date}\nGET {API_PATH} HTTP/1.1"
    signature = base64.b64encode(
        hmac.new(api_secret.encode(), signature_origin.encode(),
                 hashlib.sha256).digest()).decode()
    authorization_origin = (
        f'api_key="{api_key}", algorithm="hmac-sha256", '
        f'headers="host date request-line", signature="{signature}"')
    authorization = base64.b64encode(authorization_origin.encode()).decode()
    return (f"wss://{HOST}{API_PATH}?authorization={authorization}"
            f"&date={_rfc3986(date)}&host={HOST}")


def _parse_result(result: dict) -> str:
    return "".join(w["cw"][0]["w"] for w in result.get("ws", []))


class CloudASR:
    """讯飞流式听写客户端。密钥缺失或网络失败时 recognize() 返回 None，
    调用方回退本地 SenseVoice。"""

    def __init__(self):
        self._keys = None
        if KEYS_FILE.exists():
            try:
                d = json.loads(KEYS_FILE.read_text())
                self._keys = (d["appid"], d["api_key"], d["api_secret"])
            except (KeyError, ValueError) as e:
                print(f"[CloudASR] 密钥配置错误: {e}")

    def available(self) -> bool:
        return self._keys is not None

    def recognize(self, pcm: bytes, timeout: float = 4.0) -> str | None:
        """16kHz/16bit/单声道 PCM → 识别文本。失败返回 None。"""
        if not self.available() or not pcm:
            return None
        try:
            return asyncio.run(self._run(pcm, timeout))
        except Exception as e:
            print(f"[CloudASR] 识别失败: {e}")
            return None

    async def _run(self, pcm: bytes, timeout: float) -> str:
        import websockets
        appid, api_key, api_secret = self._keys
        url = _build_url(appid, api_key, api_secret)

        saved = {k: os.environ.pop(k) for k in _PROXY_KEYS if k in os.environ}
        try:
            async with websockets.connect(
                    url, ping_interval=None, open_timeout=timeout,
                    max_size=2 ** 20) as ws:
                return await self._exchange(ws, pcm, timeout)
        finally:
            os.environ.update(saved)

    async def _exchange(self, ws, pcm: bytes, timeout: float) -> str:
        """分帧发送并收响应直到最终帧（status=2）。"""
        # 分帧发送（status: 0=首帧带参数, 1=中间, 2=末帧）
        frames = max(1, (len(pcm) + CHUNK_BYTES - 1) // CHUNK_BYTES)
        for i in range(frames):
            chunk = pcm[i * CHUNK_BYTES:(i + 1) * CHUNK_BYTES]
            status = 0 if i == 0 else (1 if i < frames - 1 else 2)
            payload = {
                "common": {"app_id": self._keys[0]},
                "business": {
                    "language": "zh_cn", "domain": "iat",
                    "accent": "mandarin", "vad_eos": 2000,
                },
                "data": {
                    "status": status,
                    "format": "audio/L16;rate=16000",
                    "encoding": "raw",
                    "audio": base64.b64encode(chunk).decode(),
                },
            }
            await ws.send(json.dumps(payload))

        # 收响应直到最终帧（status=2）
        text = ""
        deadline = asyncio.get_running_loop().time() + timeout
        while True:
            remain = deadline - asyncio.get_running_loop().time()
            if remain <= 0:
                raise TimeoutError("讯飞响应超时")
            resp = json.loads(await asyncio.wait_for(ws.recv(), remain))
            if resp.get("code") != 0:
                raise RuntimeError(
                    f"讯飞错误码 {resp.get('code')}: {resp.get('message')}")
            data = resp.get("data", {})
            if data.get("result"):
                text = _parse_result(data["result"])
            if data.get("status") == 2:
                break
        return text.strip()
