"""视觉抽象层。

仿真模式(--sim)：从本地 resources/numbers/ 随机选图片，OCR 识别
真机模式：摄像头 + CV 模型
"""

import os
import random

from rclpy.node import Node

RESOURCES_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "resources", "numbers"
)


class VisionController:
    """数字和颜色识别。"""

    def __init__(self, node: Node, sim: bool = True):
        self._node = node
        self._sim = sim
        self._images_dir = RESOURCES_DIR

    def recognize_number(self) -> dict:
        """识别数字图片，返回 {"digit": int, "color": str}。"""
        if self._sim and os.path.isdir(self._images_dir):
            return self._recognize_local()
        elif self._sim:
            # 无本地图片时模拟
            return self._recognize_mock()
        else:
            # TODO: 真机摄像头 + CV
            self._node.get_logger().info("[Vision] 真机识别待实现")
            return self._recognize_mock()

    def _recognize_local(self) -> dict:
        """从本地图片中选一张做 OCR 识别。"""
        images = [
            f for f in os.listdir(self._images_dir)
            if f.endswith((".png", ".jpg", ".jpeg"))
        ]
        if not images:
            return self._recognize_mock()

        img_path = os.path.join(self._images_dir, random.choice(images))
        self._node.get_logger().info(f"[Vision] 识别图片: {img_path}")

        try:
            import pytesseract
            from PIL import Image
            text = pytesseract.image_to_string(
                Image.open(img_path), config="--psm 10 -c tessedit_char_whitelist=0123456789"
            ).strip()
            if text.isdigit() and 0 <= int(text) <= 9:
                return {"digit": int(text), "color": self._guess_color(img_path)}
        except ImportError:
            pass
        return self._recognize_mock()

    def _recognize_mock(self) -> dict:
        """模拟识别结果。"""
        colors = ["红色", "蓝色", "绿色", "黄色", "白色", "黑色", "紫色", "橙色"]
        result = {
            "digit": random.randint(0, 9),
            "color": random.choice(colors),
        }
        self._node.get_logger().info(f"[Vision] 模拟识别: {result}")
        return result

    def _guess_color(self, img_path: str) -> str:
        """简单颜色猜测（基于文件名或像素采样）。"""
        import numpy as np
        from PIL import Image
        try:
            img = Image.open(img_path).convert("RGB")
            arr = np.array(img.resize((50, 50)))
            avg = arr.mean(axis=(0, 1))
            r, g, b = avg
            if r > 200 and g < 100 and b < 100: return "红色"
            if r < 100 and g > 200 and b < 100: return "绿色"
            if r < 100 and g < 100 and b > 200: return "蓝色"
            if r > 200 and g > 200 and b < 100: return "黄色"
            if r > 200 and g > 200 and b > 200: return "白色"
            if r < 50 and g < 50 and b < 50: return "黑色"
            if r > 150 and g < 100 and b > 100: return "紫色"
            if r > 200 and g > 100 and b < 50: return "橙色"
        except Exception:
            pass
        return "红色"
