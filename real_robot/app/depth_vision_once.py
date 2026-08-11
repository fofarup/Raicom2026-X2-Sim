#!/usr/bin/env python3
"""Recognize one colour/number card from X2's RGB-D colour stream on PC42."""

import argparse
import json
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from task2_vision import ColourDigitRecognizer


class OneFrameRecognizer(Node):
    def __init__(self, topic: str):
        super().__init__("raicom_depth_vision_once")
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        if self.image is None:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/aima/hal/sensor/rgbd_head_front/rgb_image")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--debug", default="/tmp/raicom_depth_vision.jpg")
    args = parser.parse_args()

    rclpy.init()
    node = OneFrameRecognizer(args.topic)
    deadline = time.monotonic() + args.timeout
    while node.image is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.image is None:
        print(json.dumps({"ok": False, "error": "camera_timeout"}, ensure_ascii=False))
        node.destroy_node()
        rclpy.shutdown()
        return 2

    image = cv2.rotate(node.image, cv2.ROTATE_180)
    digit, color, confidence = ColourDigitRecognizer().recognize(image)
    debug = image.copy()
    cv2.putText(debug, f"{color} {digit} conf={confidence:.3f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
    Path(args.debug).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(args.debug, debug)
    ok = digit != "?" and color != "未知"
    print(json.dumps({"ok": ok, "digit": digit, "color": color,
                      "confidence": round(float(confidence), 4),
                      "debug": args.debug}, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
