#!/usr/bin/env python3
"""手臂关节 GUI 调参——滑块拖到满意姿势，点保存。

运行: python3 tune_gesture.py --sim
"""

import os, sys, time, threading
sys.path.insert(0, os.path.dirname(__file__))

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from core.mode_switch import ModeSwitch
from core.grasp import GraspController, ALL_ARM_JOINTS

JNAMES = ["肩俯仰","肩横滚","肩偏航","肘","腕偏航","腕俯仰","腕横滚"]
LIMITS_L = [(-3.08,2.04),(-0.061,2.993),(-2.556,2.556),(-2.356,0),(-2.556,2.556),(-0.558,0.558),(-1.571,0.724)]
LIMITS_R = [(-3.08,2.04),(-2.993,0.061),(-2.556,2.556),(-2.356,0),(-2.556,2.556),(-0.558,0.558),(-0.724,1.571)]
READY_L = [-0.35, 0.45, 0.0, -1.0, 0.0, 0.15, 0.0]
READY_R = [-0.35,-0.45, 0.0, -1.0, 0.0, 0.15, 0.0]


class ArmTuner:
    def __init__(self, sim=True):
        rclpy.init()
        self.node = Node("arm_tuner")
        self.mode = ModeSwitch(self.node)
        self.grasp = GraspController(self.node, sim=sim)
        self._running = True
        self._lock = threading.Lock()

        # 切 US
        self.node.get_logger().info("切换 US 模式...")
        if not self.mode.set("US"):
            raise RuntimeError("US 模式失败")

        # Spin 线程
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        # GUI
        self.root = tk.Tk()
        self.root.title("手臂关节调参 — 拖到满意姿势点保存")
        self.root.geometry("800x500")

        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        self.sliders = []
        self.val_labels = []
        self.deg_labels = []

        for side, limits, ready, prefix in [
            ("左臂", LIMITS_L, READY_L, "L"),
            ("右臂", LIMITS_R, READY_R, "R"),
        ]:
            f = ttk.LabelFrame(main, text=side, padding=5)
            f.pack(side="left", fill="both", expand=True, padx=5)

            for i, (name, (lo, hi), init) in enumerate(zip(JNAMES, limits, ready)):
                row = ttk.Frame(f)
                row.pack(fill="x", pady=1)

                ttk.Label(row, text=f"{prefix}{name}", width=10).pack(side="left")

                s = ttk.Scale(row, from_=lo, to=hi, value=init,
                              orient="horizontal", length=200)
                s.pack(side="left", fill="x", expand=True, padx=5)
                s.bind("<B1-Motion>", lambda e, idx=len(self.sliders): self._on_slide(idx))
                s.bind("<ButtonRelease-1>", lambda e, idx=len(self.sliders): self._on_slide(idx))

                vl = ttk.Label(row, text=f"{init:.3f}", width=7)
                vl.pack(side="left")
                dl = ttk.Label(row, text=f"({init*57.3:.0f}°)", width=7)
                dl.pack(side="left")

                self.sliders.append(s)
                self.val_labels.append(vl)
                self.deg_labels.append(dl)

        # 按钮
        btns = ttk.Frame(self.root, padding=5)
        btns.pack(fill="x")

        ttk.Button(btns, text="保存姿态", command=self._save).pack(side="left", padx=5)
        ttk.Button(btns, text="回 READY", command=self._ready).pack(side="left", padx=5)
        ttk.Button(btns, text="全部归零", command=self._zero).pack(side="left", padx=5)
        self._status = ttk.Label(btns, text="")
        self._status.pack(side="right", padx=10)

        self.root.protocol("WM_DELETE_WINDOW", self._quit)

    def _spin(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def get_angles(self):
        return [s.get() for s in self.sliders]

    def _on_slide(self, idx):
        angles = self.get_angles()
        self.val_labels[idx].config(text=f"{angles[idx]:.3f}")
        self.deg_labels[idx].config(text=f"({angles[idx]*57.3:.0f}°)")
        with self._lock:
            self.grasp.move_arm(angles, duration=0.2)

    def _save(self):
        angles = self.get_angles()
        ts = time.strftime("%H%M%S")
        fname = f"/tmp/gesture_saved_{ts}.txt"
        with open(fname, "w") as f:
            f.write(f"[{', '.join(f'{v:.4f}' for v in angles)}]\n")
        self._status.config(text=f"已保存: {fname}")
        self.node.get_logger().info(f"已保存: {fname}")

    def _ready(self):
        for i, (s, v) in enumerate(zip(self.sliders, READY_L + READY_R)):
            s.set(v)
            self.val_labels[i].config(text=f"{v:.3f}")
            self.deg_labels[i].config(text=f"({v*57.3:.0f}°)")
        self.grasp.move_arm(READY_L + READY_R, duration=0.5)
        self._status.config(text="回到 READY")

    def _zero(self):
        for i, s in enumerate(self.sliders):
            s.set(0.0)
            self.val_labels[i].config(text="0.000")
            self.deg_labels[i].config(text="(0°)")
        self.grasp.move_arm([0.0]*14, duration=0.5)
        self._status.config(text="全部归零")

    def _quit(self):
        self._running = False
        self.root.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", default=True)
    args = p.parse_args()
    ArmTuner(sim=args.sim).run()
