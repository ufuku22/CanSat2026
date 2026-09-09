#!/usr/bin/env python3
"""ARLISS用ミッションを実行する。"""

from pathlib import Path

from config import ArlissMissionConfig
from logger import start_mission_console_capture
from mission_controller import MissionController
from navigation_controller import NavigationController

TARGET_LATITUDE_DEG = 35.91912968333333    # 目標緯度
TARGET_LONGITUDE_DEG = 139.90919718333333  # 目標経度
USE_SIMPLE_SELFIE_MISSION = False          # True: 1枚撮影 / False: 露出違いで5枚撮影
STATE_FILE = Path(__file__).resolve().with_name("mission_arliss_state.json")


def main() -> None:
    navigator = NavigationController(TARGET_LATITUDE_DEG, TARGET_LONGITUDE_DEG)
    with MissionController(
        config=ArlissMissionConfig,
        navigator=navigator,
        arliss_state_file=STATE_FILE,
    ) as mission:
        mission.run_arliss_mission(
            simple_selfie=USE_SIMPLE_SELFIE_MISSION,
        )


if __name__ == "__main__":
    start_mission_console_capture()
    main()
