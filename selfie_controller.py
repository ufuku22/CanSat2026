#!/usr/bin/env python3
"""Selfie arm and ESP32-S3 Sense camera control for CanSat2026.

The FIT0579 motor is controlled through a TI DRV8838 motor driver.  DRV8838
uses a PH/EN interface: PH selects direction and EN enables the H-bridge.  PWM
is applied to EN when speed or holding torque needs to be reduced.

The camera side is expected to expose a JPEG capture endpoint over Wi-Fi, for
example the ESP32 camera web server's ``/capture`` endpoint.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Literal, Protocol
from urllib.parse import urljoin
from urllib.request import Request, urlopen


MotorDirection = Literal["forward", "reverse"]
ArmState = Literal["stowed", "deploying", "deployed", "holding", "retracting", "stopped"]
HoldMode = Literal["drive", "brake"]
DRV8838_WAKE_SECONDS = 0.001


@dataclass(frozen=True)
class Drv8838Pins:
    """GPIO pins connected to the DRV8838.

    Pin numbers are BCM GPIO numbers by default, not physical header numbers.
    ``sleep`` can be omitted when nSLEEP is tied high on the board.
    """

    phase: int
    enable: int
    sleep: int | None = None


@dataclass(frozen=True)
class SelfieControllerConfig:
    """Timing and power settings for the selfie arm."""

    motor_pins: Drv8838Pins
    esp32_base_url: str
    capture_path: str = "/capture"
    capture_dir: str | Path = "captures"
    gpio_mode: str = "BCM"
    pwm_frequency_hz: int = 1000
    forward_phase_high: bool = False
    deploy_seconds: float = 2.0
    retract_seconds: float = 2.0
    deploy_duty_cycle: float = 80.0
    retract_duty_cycle: float = 80.0
    hold_duty_cycle: float = 25.0
    hold_mode: HoldMode = "drive"
    request_timeout: float = 10.0


class MotorDriver(Protocol):
    """Small interface so a test or simulation driver can be injected later."""

    def setup(self) -> None:
        """Prepare motor outputs."""

    def run(self, direction: MotorDirection, duty_cycle: float) -> None:
        """Run the motor in one direction."""

    def brake(self) -> None:
        """Actively brake the motor if the driver supports it."""

    def coast(self) -> None:
        """Stop driving the motor."""

    def close(self) -> None:
        """Release hardware resources."""


class Drv8838MotorDriver:
    """Raspberry Pi GPIO driver for DRV8838 PH/EN motor control."""

    def __init__(
        self,
        pins: Drv8838Pins,
        pwm_frequency_hz: int = 1000,
        gpio_mode: str = "BCM",
        forward_phase_high: bool = False,
        gpio_module: Any | None = None,
    ) -> None:
        self.pins = pins
        self.pwm_frequency_hz = pwm_frequency_hz
        self.gpio_mode = gpio_mode
        self.forward_phase_high = forward_phase_high
        self.gpio = gpio_module
        self.pwm: Any | None = None
        self._is_setup = False

    def setup(self) -> None:
        if self._is_setup:
            return

        gpio = self.gpio or _load_gpio()
        self.gpio = gpio
        mode = getattr(gpio, self.gpio_mode)
        gpio.setmode(mode)
        gpio.setup(self.pins.phase, gpio.OUT)
        gpio.setup(self.pins.enable, gpio.OUT)
        if self.pins.sleep is not None:
            gpio.setup(self.pins.sleep, gpio.OUT)
            gpio.output(self.pins.sleep, gpio.HIGH)
            time.sleep(DRV8838_WAKE_SECONDS)

        self.pwm = gpio.PWM(self.pins.enable, self.pwm_frequency_hz)
        self.pwm.start(0)
        self._is_setup = True
        self.brake()

    def run(self, direction: MotorDirection, duty_cycle: float) -> None:
        self.setup()
        self._wake()
        duty = _clamp_duty_cycle(duty_cycle)
        gpio = self._require_gpio()

        if direction == "forward":
            phase_high = self.forward_phase_high
        elif direction == "reverse":
            phase_high = not self.forward_phase_high
        else:
            raise ValueError(f"unsupported motor direction: {direction!r}")

        gpio.output(self.pins.phase, gpio.HIGH if phase_high else gpio.LOW)
        self._require_pwm().ChangeDutyCycle(duty)

    def brake(self) -> None:
        self.setup()
        self._wake()
        gpio = self._require_gpio()
        self._require_pwm().ChangeDutyCycle(0)
        gpio.output(self.pins.phase, gpio.LOW)

    def coast(self) -> None:
        self.setup()
        gpio = self._require_gpio()
        self._require_pwm().ChangeDutyCycle(0)
        if self.pins.sleep is not None:
            gpio.output(self.pins.sleep, gpio.LOW)

    def close(self) -> None:
        if self.gpio and self._is_setup and self.pins.sleep is not None:
            self.gpio.output(self.pins.sleep, self.gpio.LOW)
        if self.pwm:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
            self.pwm = None
        if self.gpio and self._is_setup:
            pins = [self.pins.phase, self.pins.enable]
            if self.pins.sleep is not None:
                pins.append(self.pins.sleep)
            self.gpio.cleanup(pins)
        self._is_setup = False

    def _wake(self) -> None:
        if self.pins.sleep is None:
            return
        gpio = self._require_gpio()
        gpio.output(self.pins.sleep, gpio.HIGH)
        time.sleep(DRV8838_WAKE_SECONDS)

    def _require_gpio(self) -> Any:
        if self.gpio is None:
            raise RuntimeError("GPIO module is not loaded")
        return self.gpio

    def _require_pwm(self) -> Any:
        if self.pwm is None:
            raise RuntimeError("PWM is not initialized")
        return self.pwm


class SelfieController:
    """Control the selfie arm motor and an ESP32-S3 Sense Wi-Fi camera."""

    def __init__(
        self,
        config: SelfieControllerConfig,
        motor_driver: MotorDriver | None = None,
    ) -> None:
        self.config = config
        self.motor_driver = motor_driver or Drv8838MotorDriver(
            pins=config.motor_pins,
            pwm_frequency_hz=config.pwm_frequency_hz,
            gpio_mode=config.gpio_mode,
            forward_phase_high=config.forward_phase_high,
        )
        self.arm_state: ArmState = "stowed"
        self.last_photo_path: Path | None = None
        self.last_action_at: str | None = None

    def __enter__(self) -> "SelfieController":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def open(self) -> None:
        """Prepare the motor GPIO outputs."""
        self.motor_driver.setup()

    def close(self) -> None:
        """Stop the motor and release GPIO resources."""
        self.motor_driver.close()

    def deploy_arm(
        self,
        seconds: float | None = None,
        duty_cycle: float | None = None,
        stop_after: bool = True,
    ) -> dict[str, Any]:
        """Run the FIT0579 motor forward to deploy the arm."""
        run_seconds = self.config.deploy_seconds if seconds is None else seconds
        duty = self.config.deploy_duty_cycle if duty_cycle is None else duty_cycle
        self._run_timed_motor(
            direction="forward",
            seconds=run_seconds,
            duty_cycle=duty,
            state_while_running="deploying",
            stop_after=stop_after,
        )
        self.arm_state = "deployed" if stop_after else "deploying"
        return self.read_all()

    def hold_arm(self, duty_cycle: float | None = None) -> dict[str, Any]:
        """Keep the deployed arm in position.

        ``hold_mode="drive"`` applies low forward torque.  ``hold_mode="brake"``
        uses the motor driver's active brake input pattern.
        """
        if self.config.hold_mode == "brake":
            self.motor_driver.brake()
        else:
            duty = self.config.hold_duty_cycle if duty_cycle is None else duty_cycle
            self.motor_driver.run("forward", duty)

        self.arm_state = "holding"
        self._touch()
        return self.read_all()

    def retract_arm(
        self,
        seconds: float | None = None,
        duty_cycle: float | None = None,
        stop_after: bool = True,
    ) -> dict[str, Any]:
        """Run the FIT0579 motor in reverse to stow the arm."""
        run_seconds = self.config.retract_seconds if seconds is None else seconds
        duty = self.config.retract_duty_cycle if duty_cycle is None else duty_cycle
        self._run_timed_motor(
            direction="reverse",
            seconds=run_seconds,
            duty_cycle=duty,
            state_while_running="retracting",
            stop_after=stop_after,
        )
        self.arm_state = "stowed" if stop_after else "retracting"
        return self.read_all()

    def stop_arm(self, brake: bool = True) -> dict[str, Any]:
        """Stop motor output."""
        if brake:
            self.motor_driver.brake()
        else:
            self.motor_driver.coast()
        self.arm_state = "stopped"
        self._touch()
        return self.read_all()

    def capture_photo(
        self,
        filename: str | None = None,
        capture_url: str | None = None,
    ) -> Path:
        """Capture one JPEG photo from the ESP32 camera and save it locally."""
        url = capture_url or self._capture_url()
        request = Request(url, headers={"User-Agent": "CanSat2026-SelfieController"})

        with urlopen(request, timeout=self.config.request_timeout) as response:
            image_bytes = response.read()
            content_type = response.headers.get("Content-Type", "")

        if not image_bytes:
            raise RuntimeError("ESP32 camera returned an empty response")
        if content_type and "image" not in content_type.lower():
            raise RuntimeError(f"ESP32 camera did not return an image: {content_type}")

        capture_dir = Path(self.config.capture_dir)
        capture_dir.mkdir(parents=True, exist_ok=True)
        photo_name = filename or f"selfie_{datetime.now():%Y%m%d_%H%M%S}.jpg"
        destination = capture_dir / Path(photo_name).name
        destination.write_bytes(image_bytes)

        self.last_photo_path = destination
        self._touch()
        return destination

    def read_all(self) -> dict[str, Any]:
        """Return a compact status snapshot for Logger.register_source()."""
        return {
            "arm_state": self.arm_state,
            "last_photo_path": str(self.last_photo_path) if self.last_photo_path else None,
            "last_action_at": self.last_action_at,
            "esp32_base_url": self.config.esp32_base_url,
        }

    def _run_timed_motor(
        self,
        direction: MotorDirection,
        seconds: float,
        duty_cycle: float,
        state_while_running: ArmState,
        stop_after: bool,
    ) -> None:
        if seconds < 0:
            raise ValueError("seconds must be greater than or equal to 0")

        self.arm_state = state_while_running
        self._touch()
        self.motor_driver.run(direction, duty_cycle)
        if seconds:
            time.sleep(seconds)
        if stop_after:
            self.motor_driver.brake()

    def _capture_url(self) -> str:
        if not self.config.esp32_base_url:
            raise ValueError("esp32_base_url must be set")
        return urljoin(self.config.esp32_base_url.rstrip("/") + "/", self.config.capture_path.lstrip("/"))

    def _touch(self) -> None:
        self.last_action_at = datetime.now().isoformat(timespec="seconds")


def _load_gpio() -> Any:
    try:
        import RPi.GPIO as gpio
    except ImportError as exc:
        raise RuntimeError(
            "RPi.GPIO is required to drive the arm motor on Raspberry Pi. "
            "Install it on the Raspberry Pi or inject a test motor_driver."
        ) from exc
    return gpio


def _clamp_duty_cycle(value: float) -> float:
    return max(0.0, min(100.0, float(value)))
