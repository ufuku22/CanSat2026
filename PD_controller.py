#!/usr/bin/env python3
"""PD heading hold controller using gpiozero.

This module is based on the following.py control flow, but uses the
same gpiozero/TB6612FNG pin layout as drive_controller.py instead of
the older MotorDriver API.
"""

from __future__ import annotations

from dataclasses import dataclass
import numbers
import time
from typing import Protocol

from gpiozero import OutputDevice, PWMOutputDevice


class HeadingSensor(Protocol):
    """Minimal heading sensor interface used by the PD controller."""

    def get_heading(self) -> float:
        ...


@dataclass(frozen=True)
class PDConfig:
    base_speed: float = 80.0
    kp: float = 0.80
    kd: float = 0.0
    loop_interval: float = 0.10
    stop_ramp_steps: int = 100
    stop_ramp_interval: float = 0.03


class DifferentialDriveController:
    """TB6612FNG drive controller with left/right forward speed control."""

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
            raise TypeError("speed must be a number from 0 to 100")
        if not 0 <= speed <= 100:
            raise ValueError("speed must be in the range 0 to 100")
        return float(speed)

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("DifferentialDriveController has already been cleaned up")

    def forward_differential(self, left_speed: float, right_speed: float) -> None:
        """Drive forward with independent left and right duty cycles."""
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
        """Gradually reduce forward duty cycles, then stop."""
        steps = max(1, int(steps))
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)

        for step in range(steps - 1, -1, -1):
            ratio = step / steps
            self.forward_differential(left_speed * ratio, right_speed * ratio)
            time.sleep(interval)
        self.stop()

    def stop(self) -> None:
        """Disable outputs and stop by inertia."""
        self._ensure_open()
        self.stby.off()
        self.pwm_l.value = 0.0
        self.pwm_r.value = 0.0
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()

    def brake(self) -> None:
        """Short brake both motors."""
        self._ensure_open()
        self.pwm_l.value = 0.0
        self.pwm_r.value = 0.0
        self.stby.on()

    def cleanup(self) -> None:
        """Disable motor outputs and release gpiozero devices."""
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
    """Return signed shortest heading error in degrees (-180 to +180)."""
    return (current - target + 180.0) % 360.0 - 180.0


class PDController:
    """Keep the rover driving forward while holding the initial heading."""

    def __init__(self, driver: DifferentialDriveController, sensor: HeadingSensor, config: PDConfig | None = None):
        self.driver = driver
        self.sensor = sensor
        self.config = config or PDConfig()

    def follow_forward(self, duration_time: float, base_speed: float | None = None) -> None:
        """Drive forward for duration_time seconds using PD heading correction."""
        config = self.config
        base = _clamp_speed(config.base_speed if base_speed is None else base_speed)
        target = float(self.sensor.get_heading())
        prev_error = 0.0
        left_speed = base
        right_speed = base
        start_time = time.monotonic()

        try:
            self.driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                current = float(self.sensor.get_heading())
                error = _heading_error(current, target)
                d_error = (error - prev_error) / config.loop_interval
                correction = config.kp * error + config.kd * d_error

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
        """Shorter stop ramp variant, matching following.py's petit behavior."""
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
    """Compatibility function similar to following.py follow_forward()."""
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
    """Compatibility function similar to following.py follow_petit_forward()."""
    config = PDConfig(base_speed=base_speed, kp=kp, kd=kd, loop_interval=loop_interval)
    PDController(driver, sensor, config).follow_petit_forward(duration_time)
