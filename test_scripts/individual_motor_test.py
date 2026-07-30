#!/usr/bin/env python3
"""左右の走行モーターを片方ずつ正転させるテスト。"""

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController


SPEED = 30
RUN_SECONDS = 1

driver = DriveController()

try:
    input("Enterで左モーターを正転します")
    driver.forward_differential(SPEED, 0)
    time.sleep(RUN_SECONDS)
    driver.stop()

    input("Enterで右モーターを正転します")
    driver.forward_differential(0, SPEED)
    time.sleep(RUN_SECONDS)
    driver.stop()
finally:
    driver.cleanup()
