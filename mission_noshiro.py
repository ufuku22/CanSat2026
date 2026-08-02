#!/usr/bin/env python3
"""能代宇宙イベント用ミッションを実行する。"""

from config import NoshiroMissionConfig
from mission_controller import MissionController


def main() -> None:
    with MissionController(config=NoshiroMissionConfig) as mission:
        mission.prepare()
        mission.wait_for_release()
        mission.start_telemetry()
        mission.wait_for_landing()
        mission.start_wifi_ap()
        mission.deploy()
        mission.avoid_parachute()
        mission.clear_landing_area()
        mission.run_selfie_mission()
        mission.navigate_to_goal_area()
        mission.search_and_guide_to_red_cone()
        mission.complete()


if __name__ == "__main__":
    main()
