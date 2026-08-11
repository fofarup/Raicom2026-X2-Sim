#!/usr/bin/env python3
"""正式评分任务统一入口。

评分任务与历史文件的映射：
  1 -> task1_agent.py（自主导航与交互就位）
  2 -> task2_agent.py（基础交互）
  3 -> task3_scene_agent.py（场景交互与自主服务）
  3-grasp -> real_grasp_agent.py（真机接口预检/受保护调试）
"""

import argparse
import runpy
import sys
from pathlib import Path


TASK_FILES = {
    "1": "task1_agent.py",
    "2": "task2_agent.py",
    "3": "task3_scene_agent.py",
    "3-grasp": "real_grasp_agent.py",
    "voice-action": "voice_robot_actions.py",
}


def main():
    parser = argparse.ArgumentParser(description="RAICOM 2026 正式评分任务统一入口")
    parser.add_argument("task", choices=TASK_FILES, help="正式评分任务编号或语音内部动作")
    parser.add_argument("task_args", nargs=argparse.REMAINDER, help="传递给任务脚本的参数")
    args = parser.parse_args()

    script = Path(__file__).resolve().with_name(TASK_FILES[args.task])
    sys.argv = [str(script), *args.task_args]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
