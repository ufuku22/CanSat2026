#!/usr/bin/env python3
"""gpiozeroを使ったPD方位維持コントローラ。

following.pyの制御の流れを元にしつつ、古いMotorDriver APIではなく、
drive_controller.pyと同じgpiozero/TB6612FNGのピン配置を使う。
"""

from __future__ import annotations

from dataclasses import dataclass
import numbers
import time
from typing import Protocol

from gpiozero import OutputDevice, PWMOutputDevice


class HeadingSensor(Protocol):
    """PD制御で使う方位センサーの最小インターフェース。"""

    def get_heading(self) -> float:
        ...


@dataclass(frozen=True)
class PDConfig:
    base_speed: float = 80.0
    kp: float = 0.80
    kd: float = 0.05
    loop_interval: float = 0.10
    stop_ramp_steps: int = 100
    stop_ramp_interval: float = 0.03


class DifferentialDriveController:
    """左右の前進速度を個別に指定できるTB6612FNG用ドライブコントローラ。"""

    PWM_FREQUENCY_HZ = 100

    PIN_STBY = 21
    PIN_PWMA = 12
    PIN_AIN1 = 8
    PIN_AIN2 = 7
    PIN_PWMB = 19
    PIN_BIN1 = 25
    PIN_BIN2 = 26

    def __init__(self) -> None:
        self.stby = OutputDevice(self.PIN_STBY, active_high=True, initial_value=False)
        self.ain1 = OutputDevice(self.PIN_AIN1, active_high=True, initial_value=False)
        self.ain2 = OutputDevice(self.PIN_AIN2, active_high=True, initial_value=False)
        self.bin1 = OutputDevice(self.PIN_BIN1, active_high=True, initial_value=False)
        self.bin2 = OutputDevice(self.PIN_BIN2, active_high=True, initial_value=False)
        self.pwm_l = PWMOutputDevice(
            self.PIN_PWMA,
            active_high=True,
            initial_value=0.0,
            frequency=self.PWM_FREQUENCY_HZ,
        )
        self.pwm_r = PWMOutputDevice(
            self.PIN_PWMB,
            active_high=True,
            initial_value=0.0,
            frequency=self.PWM_FREQUENCY_HZ,
        )
        self._closed = False

    @staticmethod
    def _validate_speed(speed: float) -> float:
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speedは0から100までの数値にしてください")
        if not 0 <= speed <= 100:
            raise ValueError("speedは0から100の範囲にしてください")
        return float(speed)

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("DifferentialDriveControllerはすでにcleanup済みです")

    def forward_differential(self, left_speed: float, right_speed: float) -> None:
        """左右のデューティ比を個別に指定して前進する。"""
        self._ensure_open()
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)

        if left_speed == 0 and right_speed == 0:
            self.stop()
            return

        self.ain1.value = True
        self.ain2.value = False
        self.bin1.value = True
        self.bin2.value = False
        self.stby.on()
        self.pwm_l.value = left_speed / 100.0
        self.pwm_r.value = right_speed / 100.0

    def ramp_stop_forward(
        self,
        left_speed: float,
        right_speed: float,
        *,
        steps: int = 100,
        interval: float = 0.03,
    ) -> None:
        """前進中の左右デューティ比を少しずつ下げて停止する。"""
        steps = max(1, int(steps))
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)

        for step in range(steps - 1, -1, -1):
            ratio = step / steps
            self.forward_differential(left_speed * ratio, right_speed * ratio)
            time.sleep(interval)
        self.stop()

    def stop(self) -> None:
        """出力を切って慣性で停止する。"""
        self._ensure_open()
        self.stby.off()
        self.pwm_l.value = 0.0
        self.pwm_r.value = 0.0
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()

    def brake(self) -> None:
        """両モーターを短絡ブレーキする。"""
        self._ensure_open()
        self.pwm_l.value = 0.0
        self.pwm_r.value = 0.0
        self.stby.on()

    def cleanup(self) -> None:
        """モーター出力を止め、gpiozeroデバイスを解放する。"""
        if self._closed:
            return

        self.stop()
        for device in (self.pwm_l, self.pwm_r, self.stby, self.ain1, self.ain2, self.bin1, self.bin2):
            device.close()
        self._closed = True

    def close(self) -> None:
        self.cleanup()

    def __enter__(self) -> "DifferentialDriveController":
        return self

    def __exit__(self, *_: object) -> None:
        self.cleanup()


def _clamp_speed(speed: float) -> float:
    return max(0.0, min(100.0, float(speed)))


def _heading_error(current: float, target: float) -> float:
    """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
    # 0/360度をまたいでも最短方向の角度差になるように、-180から+180度へ丸める。
    return (current - target + 180.0) % 360.0 - 180.0


class PDController:
    """走り始めの方位を維持しながら前進するPDコントローラ。"""

    def __init__(self, driver: DifferentialDriveController, sensor: HeadingSensor, config: PDConfig | None = None):
        self.driver = driver
        self.sensor = sensor
        self.config = config or PDConfig()

    def follow_forward(self, duration_time: float, base_speed: float | None = None) -> None:
        """PD制御で方位を補正しながらduration_time秒だけ前進する。"""
        config = self.config
        base = _clamp_speed(config.base_speed if base_speed is None else base_speed)

        # 走り始めた瞬間の方位を目標方位にする。
        # この方位から右/左にどれだけずれたかをPD制御の誤差として使う。
        target = float(self.sensor.get_heading())
        prev_error = 0.0
        left_speed = base
        right_speed = base
        start_time = time.monotonic()

        try:
            self.driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                current = float(self.sensor.get_heading())

                # P制御: 現在方位と目標方位の差を補正量にする。
                # D制御: 前回ループから誤差がどれだけ変化したかを見て、曲がりすぎを抑える。
                error = _heading_error(current, target)
                d_error = (error - prev_error) / config.loop_interval
                correction = config.kp * error + config.kd * d_error

                # correctionが正なら左を遅く、右を速くして方位を戻す。
                # correctionが負なら右を遅く、左を速くして逆方向に戻す。
                left_speed = _clamp_speed(base - correction)
                right_speed = _clamp_speed(base + correction)
                self.driver.forward_differential(left_speed, right_speed)

                prev_error = error
                time.sleep(config.loop_interval)
        finally:
            self.driver.ramp_stop_forward(
                left_speed,
                right_speed,
                steps=config.stop_ramp_steps,
                interval=config.stop_ramp_interval,
            )

    def follow_petit_forward(self, duration_time: float, base_speed: float | None = None) -> None:
        """following.pyのpetit動作に合わせて、短い減速で停止する。"""
        petit_config = PDConfig(
            base_speed=self.config.base_speed,
            kp=self.config.kp,
            kd=self.config.kd,
            loop_interval=self.config.loop_interval,
            stop_ramp_steps=20,
            stop_ramp_interval=0.01,
        )
        PDController(self.driver, self.sensor, petit_config).follow_forward(duration_time, base_speed)


def follow_forward(
    driver: DifferentialDriveController,
    sensor: HeadingSensor,
    base_speed: float,
    duration_time: float,
    *,
    kp: float = 0.80,
    kd: float = 0.0,
    loop_interval: float = 0.10,
) -> None:
    """following.pyのfollow_forward()に近い互換用関数。"""
    config = PDConfig(base_speed=base_speed, kp=kp, kd=kd, loop_interval=loop_interval)
    PDController(driver, sensor, config).follow_forward(duration_time)


def follow_petit_forward(
    driver: DifferentialDriveController,
    sensor: HeadingSensor,
    base_speed: float,
    duration_time: float,
    *,
    kp: float = 0.80,
    kd: float = 0.0,
    loop_interval: float = 0.10,
) -> None:
    """following.pyのfollow_petit_forward()に近い互換用関数。"""
    config = PDConfig(base_speed=base_speed, kp=kp, kd=kd, loop_interval=loop_interval)
    PDController(driver, sensor, config).follow_petit_forward(duration_time)
