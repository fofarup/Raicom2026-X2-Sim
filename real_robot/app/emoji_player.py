#!/usr/bin/env python3
"""Small PlayEmoji client following the vendor's official synchronous example."""

import argparse

import rclpy
from aimdk_msgs.srv import PlayEmoji
from rclpy.node import Node


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("emotion_id", type=int)
    args = parser.parse_args()
    rclpy.init()
    node = Node("raicom_emoji_player")
    client = node.create_client(PlayEmoji, "/face_ui_proxy/play_emoji")
    success = False
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            print("PlayEmoji service unavailable", flush=True)
        else:
            request = PlayEmoji.Request()
            request.emotion_id = args.emotion_id
            request.mode = 1
            # The built-in only_voice agent owns the face at priority 60.
            # Competition expressions must be able to replace that default.
            request.priority = 100
            for attempt in range(8):
                request.header.header.stamp = node.get_clock().now().to_msg()
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=0.25)
                if future.done() and future.result() is not None:
                    response = future.result()
                    success = bool(response.success)
                    print(
                        f"PlayEmoji id={args.emotion_id} success={response.success} "
                        f"message={response.message}", flush=True
                    )
                    break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
