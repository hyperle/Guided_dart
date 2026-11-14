"""Host checks for the SC132GS RAW white-balance math; no hardware claims."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "tools/sc132gs_wb_calibrate.py"


class WhiteBalanceMathTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("sc132gs_wb_calibrate", SCRIPT)
        cls.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.script)

    def test_gains_use_black_subtracted_bayer_channels(self):
        # Raw phase order is [B, G1, G2, R].
        result = self.script.calculate_wb_gains(
            [110.0, 210.0, 190.0, 150.0],
            [10.0, 10.0, 10.0, 10.0],
        )
        # Signal is [100, 200, 180, 140], green reference is 190.
        self.assertAlmostEqual(result["green_signal"], 190.0)
        self.assertAlmostEqual(result["manual_gain"][0], 190.0 / 140.0)
        self.assertEqual(result["manual_gain"][1:3], [1.0, 1.0])
        self.assertAlmostEqual(result["manual_gain"][3], 190.0 / 100.0)

    def test_invalid_black_subtraction_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            self.script.calculate_wb_gains([10, 20, 20, 10], [10, 10, 10, 10])

    def test_profile_correction_is_reported_separately(self):
        result = self.script.calculate_wb_gains([100, 200, 200, 100], [0, 0, 0, 0])
        expected = [2.0 / self.script.CURRENT_PROFILE_GAIN[0], 1.0,
                    1.0, 2.0 / self.script.CURRENT_PROFILE_GAIN[3]]
        for actual, wanted in zip(result["correction_relative_to_current_profile"], expected):
            self.assertTrue(math.isclose(actual, wanted, rel_tol=1e-12))


if __name__ == "__main__":
    unittest.main()
