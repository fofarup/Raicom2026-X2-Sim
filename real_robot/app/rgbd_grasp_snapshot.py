#!/usr/bin/env python3
"""Capture a read-only RGB-D snapshot for real-X2 grasp calibration."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robot_profile import load_robot_profile


class SnapshotNode(Node):
    def __init__(self, rgb_topic: str, depth_topic: str) -> None:
        super().__init__("raicom_rgbd_grasp_snapshot")
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.rgb_stamp = None
        self.depth_stamp = None
        self.create_subscription(Image, rgb_topic, self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)

    @staticmethod
    def _stamp(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _rgb(self, msg: Image) -> None:
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.rgb_stamp = self._stamp(msg)

    def _depth(self, msg: Image) -> None:
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_stamp = self._stamp(msg)


def main() -> int:
    profile = load_robot_profile()
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="/tmp/raicom_rgbd_snapshot")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = SnapshotNode(
        profile["hardware"]["rgb_topic"], profile["hardware"]["depth_topic"]
    )
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.rgb is not None and node.depth is not None:
            if abs(node.rgb_stamp - node.depth_stamp) <= 0.15:
                break

    if node.rgb is None or node.depth is None:
        print(json.dumps({"ok": False, "error": "rgbd_timeout"}, ensure_ascii=False))
        node.destroy_node(); rclpy.shutdown()
        return 2

    # This RK4 RGB-D assembly is mounted upside down. Rotate both modalities
    # identically so pixel correspondence is preserved for depth lookup.
    rgb = cv2.rotate(node.rgb, cv2.ROTATE_180)
    depth = cv2.rotate(node.depth, cv2.ROTATE_180)
    cv2.imwrite(str(output / "rgb.png"), rgb)
    cv2.imwrite(str(output / "depth_raw.png"), depth)
    valid = depth[np.isfinite(depth) & (depth > 0)]
    stats = {
        "ok": bool(valid.size),
        "rgb_shape": list(rgb.shape),
        "depth_shape": list(depth.shape),
        "depth_dtype": str(depth.dtype),
        "stamp_delta_ms": round(abs(node.rgb_stamp - node.depth_stamp) * 1000.0, 2),
        "valid_ratio": round(float(valid.size / depth.size), 4),
        "depth_min": None if not valid.size else float(np.percentile(valid, 1)),
        "depth_median": None if not valid.size else float(np.median(valid)),
        "depth_max": None if not valid.size else float(np.percentile(valid, 99)),
        "rgb_path": str(output / "rgb.png"),
        "depth_path": str(output / "depth_raw.png"),
    }
    (output / "snapshot.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(stats, ensure_ascii=False))
    node.destroy_node(); rclpy.shutdown()
    return 0 if valid.size else 1


if __name__ == "__main__":
    raise SystemExit(main())
