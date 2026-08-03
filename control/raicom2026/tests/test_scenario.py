import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.scenario import EXPRESSIONS, GESTURES, NEEDS, parse_need, validate_draw


class ScenarioTests(unittest.TestCase):
    def test_all_required_draws(self):
        self.assertEqual((5, 5), (len(EXPRESSIONS), len(GESTURES)))
        for expression in EXPRESSIONS:
            for gesture in GESTURES:
                for hand in ("left", "right"):
                    validate_draw(expression, gesture, hand)

    def test_all_needs(self):
        self.assertEqual(3, len(NEEDS))
        for need in NEEDS:
            for keyword in need.keywords:
                self.assertEqual(need, parse_need(f"我感觉{keyword}，请帮帮我"))

    def test_ambiguous_text_is_rejected(self):
        self.assertIsNone(parse_need("今天天气不错"))


if __name__ == "__main__":
    unittest.main()
