#!/usr/bin/env python3
"""手臂关节 GUI 调参 —— 滑块 + 加减按钮 + 命名保存。

运行: python3 tune_gesture.py --sim
"""

import os, sys, time, threading
sys.path.insert(0, os.path.dirname(__file__))

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from core.mode_switch import ModeSwitch
from core.grasp import GraspController, ALL_ARM_JOINTS

# English names to avoid tkinter CJK rendering issues
JNAMES = ["shld_pitch", "shld_roll", "shld_yaw",
          "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll"]
SIDES = ["LEFT", "RIGHT"]

LIMITS = [
    [(-3.08,2.04),(-0.061,2.993),(-2.556,2.556),(-2.356,0),(-2.556,2.556),(-0.558,0.558),(-1.571,0.724)],
    [(-3.08,2.04),(-2.993,0.061),(-2.556,2.556),(-2.356,0),(-2.556,2.556),(-0.558,0.558),(-0.724,1.571)],
]
READY = [
    [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0],
    [-0.35,-0.45, 0.0, -1.0, 0.0, 0.15, 0.0],
]


class ArmTuner:
    def __init__(self, sim=True):
        rclpy.init()
        self.node = Node("arm_tuner")
        self.mode = ModeSwitch(self.node)
        self.grasp = GraspController(self.node, sim=sim)
        self._lock = threading.Lock()

        self.node.get_logger().info("Switching to US mode...")
        if not self.mode.set("US"):
            raise RuntimeError("US mode failed")

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        self.root = tk.Tk()
        self.root.title("Arm Joint Tuner")
        self.root.geometry("900x550")

        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        self.sliders = []
        self.val_labels = []

        for si, (side, limits, ready) in enumerate(zip(SIDES, LIMITS, READY)):
            f = ttk.LabelFrame(main, text=side, padding=5)
            f.pack(side="left", fill="both", expand=True, padx=5)

            for i, (name, (lo, hi), init) in enumerate(zip(JNAMES, limits, ready)):
                idx = si * 7 + i
                row = ttk.Frame(f)
                row.pack(fill="x", pady=2)

                ttk.Label(row, text=name, width=14).pack(side="left")

                # 减号
                b = ttk.Button(row, text="-", width=3,
                               command=lambda idx=idx, d=-0.02: self._step(idx, d))
                b.pack(side="left")

                # 滑块
                s = ttk.Scale(row, from_=lo, to=hi, value=init,
                              orient="horizontal", length=160,
                              command=lambda v, idx=idx: self._on_slide(idx, v))
                s.pack(side="left", fill="x", expand=True, padx=2)

                # 加号
                b = ttk.Button(row, text="+", width=3,
                               command=lambda idx=idx, d=+0.02: self._step(idx, d))
                b.pack(side="left")

                # 值
                vl = ttk.Label(row, text=f"{init:.3f} rad ({init*57.3:.0f}deg)", width=18)
                vl.pack(side="left", padx=3)

                self.sliders.append(s)
                self.val_labels.append(vl)

        # 底部：命名 + 保存 + 快捷按钮
        bottom = ttk.Frame(self.root, padding=5)
        bottom.pack(fill="x")

        ttk.Label(bottom, text="Gesture Name:").pack(side="left", padx=5)
        self.name_entry = ttk.Entry(bottom, width=20)
        self.name_entry.pack(side="left", padx=5)
        self.name_entry.insert(0, "salute")

        ttk.Button(bottom, text="Save", command=self._save).pack(side="left", padx=5)
        ttk.Button(bottom, text="READY", command=self._ready).pack(side="left", padx=5)
        ttk.Button(bottom, text="Zero", command=self._zero).pack(side="left", padx=5)
        self._status = ttk.Label(bottom, text="", foreground="green")
        self._status.pack(side="right", padx=10)

        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    def _spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def get_angles(self):
        return [s.get() for s in self.sliders]

    def _step(self, idx, delta):
        s = self.sliders[idx]
        new = round(s.get() + delta, 3)
        lo = s.cget("from")
        hi = s.cget("to")
        s.set(max(float(lo), min(float(hi), new)))
        self._update_label(idx)
        self._publish()

    def _on_slide(self, idx, val):
        self._update_label(idx)
        self._publish()

    def _update_label(self, idx):
        v = self.sliders[idx].get()
        self.val_labels[idx].config(text=f"{v:.3f} rad ({v*57.3:.0f}deg)")

    def _publish(self):
        """直接发一次目标位置，不跑轨迹。"""
        with self._lock:
            self.grasp._publish_upper_body(self.get_angles())

    def _save(self):
        name = self.name_entry.get().strip() or "gesture"
        angles = self.get_angles()
        ts = time.strftime("%H%M%S")
        fname = f"/tmp/gesture_{name}_{ts}.txt"
        with open(fname, "w") as f:
            f.write(f"# {name}\n# {time.ctime()}\n")
            f.write(f"[{', '.join(f'{v:.4f}' for v in angles)}]\n")
        self._status.config(text=f"Saved: {fname}")
        self.node.get_logger().info(f"Saved: {fname}")

    def _ready(self):
        flat = READY[0] + READY[1]
        for i, (s, v) in enumerate(zip(self.sliders, flat)):
            s.set(v)
            self._update_label(i)
        self.grasp._publish_upper_body(flat)
        self._status.config(text="READY")

    def _zero(self):
        for i, s in enumerate(self.sliders):
            s.set(0.0)
            self._update_label(i)
        self.grasp.move_arm([0.0]*14, duration=0.5)
        self._status.config(text="Zeroed")

    def _quit(self):
        self.root.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", default=True)
    ArmTuner(sim=p.parse_args().sim).run()
