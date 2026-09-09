#!/usr/bin/env python3
"""ARLISS用ミッションを実行する。"""

from config import ArlissMissionConfig
from logger import start_mission_console_capture
from mission_controller import MissionController
from navigation_controller import NavigationController
import time

TARGET_LATITUDE_DEG = 35.91912968333333    # 目標緯度
TARGET_LONGITUDE_DEG = 139.90919718333333  # 目標経度
USE_SIMPLE_SELFIE_MISSION = False          # True: 1枚撮影 / False: 露出違いで5枚撮影


def main() -> None:
    navigator = NavigationController(TARGET_LATITUDE_DEG, TARGET_LONGITUDE_DEG)
    with MissionController(config=ArlissMissionConfig, navigator=navigator) as mission:
        mission.prepare()
        time.sleep(60.0)
        mission.wait_for_release()
        mission.start_telemetry()
        mission.wait_for_landing()
        mission.start_wifi_ap()
        mission.deploy()
        mission.clear_landing_area()
        mission.run_selfie_mission(simple=USE_SIMPLE_SELFIE_MISSION)
        mission.navigate_to_goal_area()
        mission.search_for_goal()
        mission.run_selfie_mission(simple=USE_SIMPLE_SELFIE_MISSION)
        mission.guide_to_arliss_goal()
        mission.complete()


if __name__ == "__main__":
    start_mission_console_capture()
    main()
