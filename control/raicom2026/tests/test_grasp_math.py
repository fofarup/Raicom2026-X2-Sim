import math
import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.grasp import compose_arm_target, world_to_base


class GraspMathTests(unittest.TestCase):
    def test_right_arm_does_not_overwrite_left(self):
        current = list(range(14))
        target = compose_arm_target(current, "right", [100] * 7)
        self.assertEqual(current[:7], target[:7])
        self.assertEqual([100] * 7, target[7:])

    def test_left_arm_does_not_overwrite_right(self):
        current = list(range(14))
        target = compose_arm_target(current, "left", [100] * 7)
        self.assertEqual([100] * 7, target[:7])
        self.assertEqual(current[7:], target[7:])

    def test_world_to_base_rotation(self):
        xyz = world_to_base((2, 1, 0.6), (1, 1, math.pi / 2))
        self.assertAlmostEqual(0.0, xyz[0], places=6)
        self.assertAlmostEqual(-1.0, xyz[1], places=6)
        self.assertEqual(0.6, xyz[2])


if __name__ == "__main__":
    unittest.main()
