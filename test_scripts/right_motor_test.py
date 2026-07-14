#!/usr/bin/env python3
"""右モーターだけを100%で前進回転させる。"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController


driver = DriveController()
try:
    driver.forward_differential(0, 70)
finally:
    driver.cleanup()
