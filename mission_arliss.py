#!/usr/bin/env python3
"""ARLISS用ミッションを実行する。"""

from config import ArlissMissionConfig
from mission_controller import MissionController


def main() -> None:
    with MissionController(config=ArlissMissionConfig) as mission:
        mission.prepare()
        mission.wait_for_release()
        mission.start_telemetry()
        mission.wait_for_landing()
        mission.start_wifi_ap()
        mission.deploy()
        mission.clear_landing_area()
        mission.run_selfie_mission()
        mission.navigate_to_goal_area()
        mission.search_for_goal()
        mission.run_selfie_mission()
        mission.guide_to_arliss_goal()
        mission.complete()


if __name__ == "__main__":
    main()
