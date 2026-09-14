"""模擬時間・センサ入力で着地判定を確認する（実機不要）。"""

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from judge import judge_landing


class LandingPressureTest(unittest.TestCase):
    def simulate(self, pressure, accel=lambda t: 9.8, timeout=40.0):
        now = [0.0]
        sensors = Mock()
        sensors.get_imu.side_effect = lambda: {"accel_mps2": (0, 0, accel(now[0]))}
        sensors.get_environment.side_effect = lambda: {"pressure_hpa": pressure(now[0])}

        def sleep(seconds):
            now[0] += seconds

        with patch("judge.time.monotonic", side_effect=lambda: now[0]), patch(
            "judge.time.sleep", side_effect=sleep
        ):
            result = judge_landing(sensors, logger=Mock(), timeout_s=timeout)
        return result, now[0]

    @staticmethod
    def noise(t):
        return (0.4, -0.3, 0.2, -0.4, 0.1)[int(t * 2) % 5]

    def test_stationary_with_noise(self):
        self.assertEqual(self.simulate(lambda t: 880 + self.noise(t)), (True, 10.0))

    def test_descent_and_ascent_with_noise_are_rejected(self):
        for slope in (0.3, 0.5, -0.5):
            with self.subTest(slope=slope):
                self.assertFalse(self.simulate(lambda t: 880 + slope * t + self.noise(t))[0])

    def test_landing_after_descent(self):
        result, elapsed = self.simulate(lambda t: 880 + 0.5 * min(t, 20) + self.noise(t))
        self.assertTrue(result)
        self.assertGreater(elapsed, 20)
        self.assertLessEqual(elapsed, 30)

    def test_acceleration_excursion_resets_window(self):
        result, elapsed = self.simulate(lambda t: 880, lambda t: 12 if t == 9 else 9.8)
        self.assertTrue(result)
        self.assertEqual(elapsed, 19.5)

    def test_invalid_pressure_resets_window(self):
        for invalid in (float("nan"), float("inf"), 0, -1, 1200):
            with self.subTest(invalid=invalid):
                self.assertEqual(
                    self.simulate(lambda t: invalid if t == 9 else 880), (True, 19.5)
                )
                self.assertFalse(self.simulate(lambda t: invalid)[0])

    def test_isolated_pressure_spike_is_filtered(self):
        self.assertEqual(self.simulate(lambda t: 890 if t == 7 else 880), (True, 10.0))


if __name__ == "__main__":
    unittest.main()
