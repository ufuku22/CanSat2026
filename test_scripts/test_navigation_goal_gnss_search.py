#!/usr/bin/env python3
"""GNSSゴール周辺探索の単体テスト。"""

from __future__ import annotations

from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import cv2  # noqa: F401
except ImportError:
    sys.modules["cv2"] = types.ModuleType("cv2")

from navigation_controller import NavigationController
from navigation_goal import search_around_gnss_goal


class FakeDriver:
    def __init__(self):
        self.stop_count = 0

    def stop(self):
        self.stop_count += 1


class FakeSensorManager:
    def __init__(self):
        self.capture_count = 0

    def get_gnss(self):
        return {
            "has_fix": True,
            "latitude_deg": 35.0,
            "longitude_deg": 139.0,
        }

    def capture_front_frame(self):
        self.capture_count += 1
        return object()


class FakeImageProcessor:
    RED_HSV_RANGES = []

    def __init__(self, red_ratios):
        self.red_ratios = iter(red_ratios)

    def detect_color(self, frame, **kwargs):
        red_ratio = float(next(self.red_ratios))
        return {
            "is_color_detected": red_ratio >= kwargs["color_threshold"],
            "total_color_ratio": red_ratio,
            "color_peak_column_x": None,
            "color_peak_center_offset_ratio": None,
            "color_peak_columns": [],
            "color_peak_count": 0,
            "color_mask": object(),
            "image_width": 1920,
            "image_height": 1080,
            "reason": "",
        }


class FakeTargetNavigationController:
    created = []

    def __init__(self, target_latitude_deg, target_longitude_deg):
        self.target_latitude_deg = target_latitude_deg
        self.target_longitude_deg = target_longitude_deg
        self.follow_called = False
        self.rotation_angles = []
        self.__class__.created.append(self)

    def follow_target(self, driver, sensor_manager, **kwargs):
        self.follow_called = True
        return True

    def rotate_by_angle(
        self,
        driver,
        sensor_manager,
        angle_deg,
        **kwargs,
    ):
        self.rotation_angles.append(float(angle_deg))
        return {
            "target_angle_deg": float(angle_deg),
            "rotated_angle_deg": float(angle_deg),
            "reached": True,
        }


class NavigationGoalGnssSearchTest(unittest.TestCase):
    def setUp(self):
        FakeTargetNavigationController.created.clear()
        self.driver = FakeDriver()
        self.sensors = FakeSensorManager()
        self.original_navigator = NavigationController()

    def run_search(self, red_ratios, *, scan_angle_deg):
        with (
            patch("navigation_goal.random.uniform", return_value=90.0),
            patch(
                "navigation_goal.NavigationController",
                FakeTargetNavigationController,
            ),
        ):
            return search_around_gnss_goal(
                self.original_navigator,
                self.driver,
                self.sensors,
                search_distance_m=10.0,
                red_ratio_threshold=0.10,
                scan_angle_deg=scan_angle_deg,
                processor=FakeImageProcessor(red_ratios),
            )

    def test_random_destination_is_ten_meters_east(self):
        result = self.run_search([0.20], scan_angle_deg=60.0)
        target = result["target_gnss"]
        destination_checker = NavigationController(
            target["latitude_deg"],
            target["longitude_deg"],
        )

        self.assertAlmostEqual(
            destination_checker.distance_to_target_m(35.0, 139.0),
            10.0,
            places=5,
        )
        self.assertAlmostEqual(
            destination_checker.bearing_to_target(35.0, 139.0),
            90.0,
            places=5,
        )
        self.assertTrue(FakeTargetNavigationController.created[0].follow_called)

    def test_search_stops_rotating_when_red_is_detected(self):
        result = self.run_search(
            [0.01, 0.02, 0.30],
            scan_angle_deg=60.0,
        )
        navigator = FakeTargetNavigationController.created[0]

        self.assertTrue(result["red_detected"])
        self.assertEqual(
            result["scan_result"]["rotation_completed_deg"],
            120.0,
        )
        self.assertEqual(navigator.rotation_angles, [60.0, 60.0])
        self.assertEqual(self.sensors.capture_count, 3)
        self.assertGreaterEqual(self.driver.stop_count, 1)

    def test_search_completes_exactly_360_degrees_without_red(self):
        result = self.run_search(
            [0.01, 0.01, 0.01, 0.01],
            scan_angle_deg=100.0,
        )
        navigator = FakeTargetNavigationController.created[0]

        self.assertFalse(result["red_detected"])
        self.assertEqual(
            result["scan_result"]["rotation_completed_deg"],
            360.0,
        )
        self.assertEqual(
            navigator.rotation_angles,
            [100.0, 100.0, 100.0, 60.0],
        )


if __name__ == "__main__":
    unittest.main()
