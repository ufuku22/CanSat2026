"""
Enterキーで溶断回路のON/OFFを切り替えるテストスクリプト。
"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import fusing
from gpiozero import OutputDevice


def main():
    """Enterキー入力ごとに溶断回路のON/OFFを切り替える。"""
    output = OutputDevice(fusing.GPIO_PIN, active_high=True, initial_value=False)
    is_on = False

    print("=== 溶断回路ON/OFFテスト開始 ===")
    print("Enter: ON/OFF切り替え")
    print("q + Enter: 終了")

    try:
        while True:
            command = input("待機中 > ").strip().lower()
            if command == "q":
                break

            if is_on:
                output.off()
                is_on = False
                print("溶断回路: OFF")
            else:
                output.on()
                is_on = True
                print("溶断回路: ON")

    except KeyboardInterrupt:
        print("\n溶断回路ON/OFFテストを中断しました")

    finally:
        output.off()
        output.close()
        print("溶断回路: OFF")
        print("=== 溶断回路ON/OFFテスト終了 ===")


if __name__ == "__main__":
    main()
