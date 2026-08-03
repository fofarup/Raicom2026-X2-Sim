#!/usr/bin/env python3
"""Verify docked pregrasp, contact and lift poses for all hand/object pairs."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "control" / "raicom2026" / "ik_sdk"))
from x2_ik_sdk import X2ArmIKSolver, X2IKConfig


def main():
    solver = X2ArmIKSolver(X2IKConfig.default_omnipicker())
    print("ready_fk:", {side: solver.fk_xyz(side) for side in ("left", "right")})
    failures = []
    object_centres = {"药盒": 0.59, "水杯": 0.59, "面包": 0.56}
    pelvis_z = 0.67
    for side in ("left", "right"):
        lateral = 0.25 if side == "left" else -0.25
        for object_name, centre_z in object_centres.items():
            contact_z = centre_z + 0.19 - pelvis_z
            poses = {
                "pregrasp": [0.15, lateral, contact_z + 0.03],
                "contact": [0.25, lateral, contact_z],
                "lift": [0.25, lateral, contact_z + 0.15],
            }
            for phase, target in poses.items():
                result = solver.solve_position(side, target)
                print(f"{side} {object_name} {phase} target={target} "
                      f"success={result.success} error={result.error_norm:.5f}")
                if not result.success:
                    failures.append((side, object_name, phase, result.error_norm))
    if failures:
        raise SystemExit(f"FAIL: {failures}")
    print("PASS: both arms reach every pregrasp/contact/lift pose")


if __name__ == "__main__":
    main()
