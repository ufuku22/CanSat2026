#!/usr/bin/env python3
"""開始時の気圧を基準に、BME280から相対高度を表示する。"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import BME280, I2C_BUS, SMBus


READ_INTERVAL_S = 1.0
MOVING_AVERAGE_SAMPLES = 10
DRY_AIR_GAS_CONSTANT = 287.05  # J/(kg*K)
GRAVITY_MPS2 = 9.80665


def input_air_temperature_c() -> float:
    while True:
        try:
            temperature_c = float(input("外気温 [°C] を入力してください: "))
        except ValueError:
            print("数値で入力してください。")
            continue
        if temperature_c <= -273.15:
            print("-273.15°Cより高い値を入力してください。")
            continue
        return temperature_c


def relative_altitude_m(
    reference_pressure_hpa: float,
    pressure_hpa: float,
    air_temperature_c: float,
) -> float:
    temperature_k = air_temperature_c + 273.15
    return (
        DRY_AIR_GAS_CONSTANT
        * temperature_k
        / GRAVITY_MPS2
        * math.log(reference_pressure_hpa / pressure_hpa)
    )


def main() -> None:
    air_temperature_c = input_air_temperature_c()
    if SMBus is None:
        raise SystemExit("Raspberry Pi上でsmbus2またはsmbusが必要です。")

    bus = SMBus(I2C_BUS)
    environment = BME280(bus)

    try:
        environment.setup()
        # センサ設定直後の測定完了を待ってから、開始地点の気圧を記録する。
        time.sleep(1.0)
        reference_pressure_hpa = environment.read()["pressure_hpa"]

        print(f"基準気圧: {reference_pressure_hpa:.2f} hPa = 0.00 m")
        print(f"計算用外気温: {air_temperature_c:.1f} °C")
        print("1秒ごとに相対高度を表示します。Ctrl+Cで終了します。")

        start_time = time.monotonic()
        next_read_time = start_time
        altitude_history: deque[float] = deque(maxlen=MOVING_AVERAGE_SAMPLES)

        while True:
            now = time.monotonic()
            if now < next_read_time:
                time.sleep(next_read_time - now)

            pressure_hpa = environment.read()["pressure_hpa"]
            elapsed_s = time.monotonic() - start_time
            altitude_m = relative_altitude_m(
                reference_pressure_hpa,
                pressure_hpa,
                air_temperature_c,
            )
            altitude_history.append(altitude_m)
            average_altitude_m = sum(altitude_history) / len(altitude_history)
            print(
                f"{elapsed_s:7.1f} s | "
                f"{pressure_hpa:8.2f} hPa | "
                f"相対高度 {altitude_m:8.2f} m | "
                f"約10秒平均 {average_altitude_m:8.2f} m"
            )

            next_read_time += READ_INTERVAL_S
            if next_read_time < time.monotonic():
                next_read_time = time.monotonic()

    except KeyboardInterrupt:
        print("\n測定を終了しました。")
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    main()
