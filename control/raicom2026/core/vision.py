"""数字/颜色视觉识别（完全离线）。

真机模式：从X2头部RGBD相机拍照识别
  话题: /aima/hal/sensor/rgbd_head_front/rgb_image
  消息: sensor_msgs/Image (rgb8)
  QoS: BEST_EFFORT + KEEP_LAST(5)

仿真模式：读本地图片文件（resources/numbers/number_*.png）

用法:
  # 仿真
  vc = VisionController(node, sim=True, image_path="number_01.png")
  result = vc.recognize_number()  # {'digit': 5, 'color': '红色'}

  # 真机
  vc = VisionController(node, sim=False)
  result = vc.recognize_number()  # 拍一帧 → 识别 → 返回
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image as ROSImage

RESOURCES_DIR = Path(__file__).resolve().parents[1] / "resources" / "numbers"

# X2 头部 RGBD 相机
CAMERA_TOPIC = "/aima/hal/sensor/rgbd_head_front/rgb_image"

# 15 种官方颜色色板
COLOR_PALETTE = {
    "粉色": (216, 27, 96), "青色": (0, 172, 193), "绿色": (67, 160, 71),
    "黄色": (249, 168, 37), "紫色": (142, 36, 170), "深橙色": (244, 81, 30),
    "蓝绿色": (0, 137, 123), "蓝色": (30, 136, 229), "浅蓝色": (3, 155, 229),
    "红色": (229, 57, 53), "深紫色": (94, 53, 177), "靛蓝色": (57, 73, 171),
    "黄绿色": (192, 202, 51), "橙色": (251, 140, 0), "浅绿色": (124, 179, 66),
}

# 数字模板文件编号映射（官方63张素材）
DIGIT_FILE_RANGES = {
    0: range(1, 7), 1: range(7, 14), 2: range(14, 20), 3: range(20, 25),
    4: range(25, 31), 5: range(31, 39), 6: range(39, 45),
    7: range(45, 51), 8: range(51, 56), 9: range(56, 64),
}


def expected_digit_for_filename(path: str | os.PathLike[str]) -> int:
    """官方样图标签。只在建立模板库时读取，不参与待测图推理。"""
    number = int(Path(path).stem.rsplit("_", 1)[1])
    for digit, numbers in DIGIT_FILE_RANGES.items():
        if number in numbers:
            return digit
    raise ValueError(f"不是官方数字样图编号: {path}")


def _foreground(rgb: np.ndarray, threshold: float = 35.0) -> np.ndarray:
    """前景分割：与白色背景的L2距离 > threshold 即为数字区域。"""
    mask = np.linalg.norm(255.0 - rgb.astype(np.float32), axis=2) > threshold
    if not mask.any():
        raise ValueError("图像中没有检测到数字前景")
    return mask


def normalized_glyph(image: Image.Image, size: int = 64) -> np.ndarray:
    """前景提取 → 裁剪 → 缩放到 size×size 画布。"""
    rgb = np.asarray(image.convert("RGB"))
    mask = _foreground(rgb)
    ys, xs = np.nonzero(mask)
    crop = (mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1] * 255).astype(np.uint8)
    h, w = crop.shape
    scale = min((size - 8) / max(w, 1), (size - 8) / max(h, 1))
    nearest = getattr(getattr(Image, "Resampling", Image), "NEAREST")
    resized = Image.fromarray(crop).resize(
        (max(1, round(w * scale)), max(1, round(h * scale))), nearest
    )
    arr = np.asarray(resized, dtype=np.float32) / 255.0
    canvas = np.zeros((size, size), dtype=np.float32)
    y0, x0 = (size - arr.shape[0]) // 2, (size - arr.shape[1]) // 2
    canvas[y0:y0 + arr.shape[0], x0:x0 + arr.shape[1]] = arr
    return canvas


def recognize_color(image: Image.Image) -> str:
    """取前景像素的中值颜色，匹配最近色板颜色。"""
    rgb = np.asarray(image.convert("RGB"))
    observed = np.median(rgb[_foreground(rgb)], axis=0)
    return min(COLOR_PALETTE, key=lambda name: float(
        np.linalg.norm(observed - np.asarray(COLOR_PALETTE[name]))))


def _ros_image_to_pil(msg: ROSImage) -> Image.Image:
    """sensor_msgs/Image (rgb8) → PIL Image。"""
    if msg.encoding not in ("rgb8", "rgba8"):
        raise ValueError(f"不支持的图像编码: {msg.encoding}")
    channels = 3 if msg.encoding == "rgb8" else 4
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    return Image.fromarray(arr, "RGB" if channels == 3 else "RGBA")


class NumberRecognizer:
    """数字模板匹配识别器。用63张官方素材建立模板库。"""

    def __init__(self, reference_dir: str | os.PathLike[str] = RESOURCES_DIR):
        paths = sorted(Path(reference_dir).glob("number_*.png"))
        if not paths:
            raise FileNotFoundError(f"找不到数字模板: {reference_dir}")
        self._templates = [(expected_digit_for_filename(path),
                            normalized_glyph(Image.open(path)))
                           for path in paths]

    def recognize(self, image: str | os.PathLike[str] | Image.Image) -> dict:
        """输入图片路径或 PIL Image，返回 {'digit': int, 'color': str}。"""
        if isinstance(image, (str, os.PathLike)):
            image = Image.open(image)
        glyph = normalized_glyph(image)
        digit, _ = min(self._templates,
                       key=lambda item: float(np.mean(np.square(glyph - item[1]))))
        return {"digit": digit, "color": recognize_color(image)}


class VisionController:
    """视觉控制器：仿真读文件 / 真机拍照。

    sim=True  → 读本地 number_*.png（仿真模式）
    sim=False → 订阅相机话题，拍一帧识别（真机模式）
    """

    def __init__(self, node: Node, sim: bool = True,
                 image_path: str | None = None):
        self._node = node
        self._sim = sim
        self._image_path = image_path
        self._recognizer = NumberRecognizer()

        # 真机：订阅 X2 头部 RGBD 相机
        if not sim:
            self._latest_frame: ROSImage | None = None
            qos = QoSPresetProfiles.SENSOR_DATA.value
            self._sub = node.create_subscription(
                ROSImage, CAMERA_TOPIC, self._on_frame, qos)
            node.get_logger().info(f"[Vision] 已订阅相机: {CAMERA_TOPIC}")

    def _on_frame(self, msg: ROSImage):
        self._latest_frame = msg

    def _capture(self, timeout: float = 3.0) -> Image.Image:
        """等待一张相机帧，转为 PIL Image。"""
        if self._sim:
            raise RuntimeError("仿真模式不支持拍照，请指定 image_path")

        # 等第一帧到达
        deadline = time.monotonic() + timeout
        while self._latest_frame is None:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"相机 {CAMERA_TOPIC} 在 {timeout}s 内未收到图像，"
                    "请检查相机是否已启动")
            time.sleep(0.05)

        frame = self._latest_frame
        self._latest_frame = None  # 清空，下次重新拍
        self._node.get_logger().info(
            f"[Vision] 拍照完成: {frame.width}x{frame.height} {frame.encoding}")
        return _ros_image_to_pil(frame)

    def recognize_number(self, image_path: str | None = None) -> dict:
        """识别数字和颜色。

        仿真模式：读本地文件（--number-image 或 RAICOM_NUMBER_IMAGE）
        真机模式：拍一帧相机图像 → 识别
        """
        if self._sim:
            selected = (image_path or self._image_path
                        or os.environ.get("RAICOM_NUMBER_IMAGE"))
            if not selected:
                raise RuntimeError(
                    "未提供数字图片；请设置 --number-image 或 RAICOM_NUMBER_IMAGE")
            path = Path(selected)
            if not path.is_absolute():
                path = RESOURCES_DIR / path
            image = Image.open(path)
            self._node.get_logger().info(f"[Vision] 读文件: {path.name}")
        else:
            image = self._capture()

        result = self._recognizer.recognize(image)
        self._node.get_logger().info(f"[Vision] 识别结果: {result}")
        return result
