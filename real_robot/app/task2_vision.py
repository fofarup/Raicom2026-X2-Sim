#!/usr/bin/env python3
"""Task 2 colour-digit recognition without OCR or network services."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


COLOR_REFS = {
    "红色": 0,
    "橙色": 18,
    "黄色": 30,
    "绿色": 60,
    "青色": 90,
    "蓝色": 115,
    "紫色": 145,
    "粉色": 165,
}


def foreground_mask(image: np.ndarray) -> np.ndarray:
    """Return the largest non-white, sufficiently saturated component."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # The supplied cards use a coloured glyph on a nearly white background.
    mask = (hsv[:, :, 1] >= 35).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return np.zeros(mask.shape, np.uint8)
    image_area = image.shape[0] * image.shape[1]
    # Ignore large coloured walls/banners.  A hand-held digit occupies a
    # compact component near the camera centre, never most of the frame.
    candidates = []
    cx0, cy0 = image.shape[1] / 2.0, image.shape[0] / 2.0
    for idx in range(1, count):
        x, y, w, h, area = stats[idx]
        if not image_area * 0.002 <= area <= image_area * 0.12:
            continue
        cx, cy = x + w / 2.0, y + h / 2.0
        aspect = w / max(1.0, float(h))
        fill_ratio = area / max(1.0, float(w * h))
        if (min(w, h) < 30 or not 0.25 <= aspect <= 2.2 or
                fill_ratio < 0.12):
            continue
        if not (0.10 * image.shape[1] <= cx <= 0.90 * image.shape[1] and
                0.10 * image.shape[0] <= cy <= 0.90 * image.shape[0]):
            continue
        centre_penalty = ((cx - cx0) / image.shape[1]) ** 2 + ((cy - cy0) / image.shape[0]) ** 2
        # Competition digits are printed on a white card.  Prefer a compact
        # coloured component surrounded by bright, low-saturation pixels;
        # this rejects the large blue venue banners behind the card.
        pad_x, pad_y = max(12, w), max(12, h)
        x0, x1 = max(0, x - pad_x), min(image.shape[1], x + w + pad_x)
        y0, y1 = max(0, y - pad_y), min(image.shape[0], y + h + pad_y)
        surround = hsv[y0:y1, x0:x1]
        white_ratio = float(np.mean(
            (surround[:, :, 1] < 45) & (surround[:, :, 2] > 140)
        ))
        if white_ratio < 0.30:
            continue
        score = 3.0 * white_ratio - 5.0 * centre_penalty
        candidates.append((score, idx))
    if not candidates:
        return np.zeros(mask.shape, np.uint8)
    idx = max(candidates)[1]
    return (labels == idx).astype(np.uint8) * 255


def normalized_glyph(mask: np.ndarray, width: int = 64, height: int = 96) -> np.ndarray | None:
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, w, h = cv2.boundingRect(points)
    roi = mask[y:y + h, x:x + w]
    margin = 4
    scale = min((width - 2 * margin) / w, (height - 2 * margin) / h)
    resized = cv2.resize(
        roi,
        (max(1, round(w * scale)), max(1, round(h * scale))),
        interpolation=cv2.INTER_AREA,
    )
    resized = (resized >= 128).astype(np.uint8) * 255
    canvas = np.zeros((height, width), np.uint8)
    yy = (height - resized.shape[0]) // 2
    xx = (width - resized.shape[1]) // 2
    canvas[yy:yy + resized.shape[0], xx:xx + resized.shape[1]] = resized
    return canvas


class ColourDigitRecognizer:
    def __init__(self, template_path: str | Path | None = None):
        path = Path(template_path) if template_path else Path(__file__).with_name("task2_digit_templates.npz")
        data = np.load(path)
        self.templates = data["templates"].astype(np.uint8)
        self.labels = data["labels"].astype(np.uint8)

    def recognize(self, image: np.ndarray) -> tuple[str, str, float]:
        mask = foreground_mask(image)
        glyph = normalized_glyph(mask)
        if glyph is None:
            return "?", "未知", 0.0

        # A small translation search absorbs imperfect card/camera cropping.
        best_score = float("inf")
        best_label = -1
        for template, label in zip(self.templates, self.labels):
            for dx in (-2, 0, 2):
                for dy in (-2, 0, 2):
                    moved = np.roll(glyph, (dy, dx), axis=(0, 1))
                    score = float(np.mean(cv2.absdiff(moved, template))) / 255.0
                    if score < best_score:
                        best_score, best_label = score, int(label)

        confidence = max(0.0, 1.0 - best_score / 0.35)
        digit = str(best_label) if best_label >= 0 and confidence >= 0.30 else "?"
        return digit, self.detect_color(image, mask), confidence

    @staticmethod
    def detect_color(image: np.ndarray, mask: np.ndarray) -> str:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        pixels = hsv[mask > 0]
        if len(pixels) == 0:
            return "未知"
        hue = float(np.median(pixels[:, 0]))
        sat = float(np.median(pixels[:, 1]))
        val = float(np.median(pixels[:, 2]))
        if val < 55:
            return "黑色"
        if sat < 35:
            return "灰色" if val < 220 else "白色"

        def circular_distance(ref: float) -> float:
            delta = abs(hue - ref)
            return min(delta, 180.0 - delta)

        return min(COLOR_REFS, key=lambda name: circular_distance(COLOR_REFS[name]))
