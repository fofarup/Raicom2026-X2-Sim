#!/usr/bin/env python3
"""Execute one X2 preset motion using the vendor's cross-board retry pattern."""

import argparse

import rclpy
from aimdk_msgs.msg import McActionCommand, McControlArea, McPresetMotion, RequestHeader
from aimdk_msgs.srv import SetMcAction, SetMcPresetMotion
from rclpy.node import Node


def call_with_retry(node, client, request, label):
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"{label}: service unavailable", flush=True)
        return None
    future = None
    for attempt in range(8):
        request.header.stamp = node.get_clock().now().to_msg()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=0.25)
        if future.done() and future.result() is not None:
            return future.result()
        print(f"{label}: retry {attempt + 1}/8", flush=True)
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("area", type=int)
    parser.add_argument("motion", type=int)
    args = parser.parse_args()

    rclpy.init()
    node = Node("raicom_motion_player")
    success = False
    try:
        action_client = node.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
        action = SetMcAction.Request()
        action.header = RequestHeader()
        action.source = "raicom_task2"
        action.command = McActionCommand(action_desc="STAND_DEFAULT")
        action_response = call_with_retry(node, action_client, action, "SetMcAction SD")
        if action_response is None:
            print("SetMcAction SD: no response; motion cancelled", flush=True)
            return
        common = action_response.response
        print(
            f"SetMcAction SD: code={common.header.code} state={common.status.value} "
            f"message={common.message}", flush=True
        )
        if common.header.code != 0 or common.status.value != 1:
            return

        motion_client = node.create_client(
            SetMcPresetMotion, "/aimdk_5Fmsgs/srv/SetMcPresetMotion"
        )
        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.area = McControlArea(value=args.area)
        request.motion = McPresetMotion(value=args.motion)
        request.interrupt = False
        response = call_with_retry(node, motion_client, request, "SetMcPresetMotion")
        if response is None:
            print("SetMcPresetMotion: no response", flush=True)
            return
        result = response.response
        print(
            f"SetMcPresetMotion: code={result.header.code} state={result.state.value} "
            f"task_id={result.task_id}", flush=True
        )
        success = result.header.code == 0 and result.state.value in (1, 200, 300, 400)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
