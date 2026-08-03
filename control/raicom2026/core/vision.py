"""数字/颜色视觉识别（完全离线）。"""
from __future__ import annotations

import os
from pathlib import Path
import numpy as np
from PIL import Image
from rclpy.node import Node

RESOURCES_DIR = Path(__file__).resolve().parents[1] / "resources" / "numbers"
COLOR_PALETTE = {
    "粉色": (216, 27, 96), "青色": (0, 172, 193), "绿色": (67, 160, 71),
    "黄色": (249, 168, 37), "紫色": (142, 36, 170), "深橙色": (244, 81, 30),
    "蓝绿色": (0, 137, 123), "蓝色": (30, 136, 229), "浅蓝色": (3, 155, 229),
    "红色": (229, 57, 53), "深紫色": (94, 53, 177), "靛蓝色": (57, 73, 171),
    "黄绿色": (192, 202, 51), "橙色": (251, 140, 0), "浅绿色": (124, 179, 66),
}
DIGIT_FILE_RANGES = {
    0: range(1, 7), 1: range(7, 14), 2: range(14, 20), 3: range(20, 25),
    4: range(25, 31), 5: range(31, 39), 6: range(39, 45),
    7: range(45, 51), 8: range(51, 56), 9: range(56, 64),
}


def expected_digit_for_filename(path: str | os.PathLike[str]) -> int:
    """官方样图标签。只在建立模板库和测试时读取，不参与待测图推理。"""
    number = int(Path(path).stem.rsplit("_", 1)[1])
    for digit, numbers in DIGIT_FILE_RANGES.items():
        if number in numbers:
            return digit
    raise ValueError(f"不是官方数字样图编号: {path}")


def _foreground(rgb: np.ndarray, threshold: float = 35.0) -> np.ndarray:
    mask = np.linalg.norm(255.0 - rgb.astype(np.float32), axis=2) > threshold
    if not mask.any():
        raise ValueError("图像中没有检测到数字前景")
    return mask


def normalized_glyph(image: Image.Image, size: int = 64) -> np.ndarray:
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
    rgb = np.asarray(image.convert("RGB"))
    observed = np.median(rgb[_foreground(rgb)], axis=0)
    return min(COLOR_PALETTE, key=lambda name: float(
        np.linalg.norm(observed - np.asarray(COLOR_PALETTE[name]))))


class NumberRecognizer:
    def __init__(self, reference_dir: str | os.PathLike[str] = RESOURCES_DIR):
        paths = sorted(Path(reference_dir).glob("number_*.png"))
        if not paths:
            raise FileNotFoundError(f"找不到数字模板: {reference_dir}")
        self._templates = [(expected_digit_for_filename(path), normalized_glyph(Image.open(path)))
                           for path in paths]

    def recognize(self, path: str | os.PathLike[str]) -> dict:
        image = Image.open(path)
        glyph = normalized_glyph(image)
        digit, _ = min(self._templates,
                       key=lambda item: float(np.mean(np.square(glyph - item[1]))))
        return {"digit": digit, "color": recognize_color(image)}


class VisionController:
    def __init__(self, node: Node, sim: bool = True, image_path: str | None = None):
        self._node, self._sim, self._image_path = node, sim, image_path
        self._recognizer = NumberRecognizer()

    def recognize_number(self, image_path: str | None = None) -> dict:
        selected = image_path or self._image_path or os.environ.get("RAICOM_NUMBER_IMAGE")
        if not selected:
            raise RuntimeError("未提供数字图片；请设置 --number-image 或 RAICOM_NUMBER_IMAGE")
        path = Path(selected)
        if not path.is_absolute():
            path = RESOURCES_DIR / path
        result = self._recognizer.recognize(path)
        self._node.get_logger().info(f"[Vision] {path.name}: {result}")
        return result
