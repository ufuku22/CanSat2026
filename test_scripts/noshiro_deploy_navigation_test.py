#!/usr/bin/env python3
"""能代ミッションの展開からGNSSゴール到達までを実機でテストする。"""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import NoshiroMissionConfig  # noqa: E402
from logger import Logger  # noqa: E402
from mission_controller import MissionController  # noqa: E402
from navigation_controller import NavigationController  # noqa: E402


TARGET_LATITUDE_DEG = 35.9184062    # 目標緯度
TARGET_LONGITUDE_DEG = 139.9079523  # 目標経度


class _NoOpHistory:
    """制御履歴をファイルに保存しないためのテスト用実装。"""

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def main() -> None:
    logger = Logger(log_to_file=False)
    navigator = NavigationController(
        TARGET_LATITUDE_DEG,
        TARGET_LONGITUDE_DEG,
        logger=logger,
    )
    mission = MissionController(
        config=NoshiroMissionConfig,
        logger=logger,
        navigator=navigator,
        history=_NoOpHistory(),
    )
    mission.communication_logger.log_to_file = False

    with mission:
        mission.prepare()
        mission.deploy()
        mission.clear_landing_area()
        mission.navigate_to_goal_area()


if __name__ == "__main__":
    main()
