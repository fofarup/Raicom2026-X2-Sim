import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.vision import NumberRecognizer, expected_digit_for_filename, recognize_color
from PIL import Image


class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.images = sorted((ROOT / "resources" / "numbers").glob("number_*.png"))
        cls.recognizer = NumberRecognizer(cls.images[0].parent)

    def test_all_63_official_images(self):
        self.assertEqual(63, len(self.images))
        for path in self.images:
            with self.subTest(path=path.name):
                result = self.recognizer.recognize(path)
                self.assertEqual(expected_digit_for_filename(path), result["digit"])
                self.assertTrue(result["color"])

    def test_background_is_not_reported_as_white(self):
        for path in self.images:
            with self.subTest(path=path.name):
                self.assertNotEqual("白色", recognize_color(Image.open(path)))


if __name__ == "__main__":
    unittest.main()
