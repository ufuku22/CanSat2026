#!/usr/bin/env python3
"""Simple DRV8838 motor control from a Raspberry Pi.

Default wiring uses BCM GPIO numbers:
  Raspberry Pi GPIO23 -> DRV8838 PH
  Raspberry Pi GPIO24 -> DRV8838 EN
  Raspberry Pi GPIO25 -> DRV8838 nSLEEP
  Raspberry Pi GND    -> DRV8838 GND

Connect DRV8838 OUT1/OUT2 to the motor. Supply VM from the motor battery or
motor power source, and VCC from the Raspberry Pi 3.3 V pin.
"""

from __future__ import annotations

import argparse
import signal
import time

from gpiozero import OutputDevice, PWMOutputDevice


DEFAULT_PH_PIN = 23
DEFAULT_EN_PIN = 24
DEFAULT_SLEEP_PIN = 25
DEFAULT_PWM_FREQUENCY_HZ = 1000


class DRV8838Motor:
    """PH/EN interface for the DRV8838.

    DRV8838 logic:
      nSLEEP=0: coast/sleep
      nSLEEP=1, EN=0: brake
      nSLEEP=1, PH=0, EN=1: forward
      nSLEEP=1, PH=1, EN=1: reverse
    """

    def __init__(
        self,
        ph_pin: int = DEFAULT_PH_PIN,
        en_pin: int = DEFAULT_EN_PIN,
        sleep_pin: int = DEFAULT_SLEEP_PIN,
        pwm_frequency_hz: int = DEFAULT_PWM_FREQUENCY_HZ,
    ) -> None:
        self.ph = OutputDevice(ph_pin, active_high=True, initial_value=False)
        self.en = PWMOutputDevice(
            en_pin,
            active_high=True,
            initial_value=0.0,
            frequency=pwm_frequency_hz,
        )
        self.sleep = OutputDevice(sleep_pin, active_high=True, initial_value=False)

    def wake(self) -> None:
        self.sleep.on()
        time.sleep(0.002)

    def forward(self, speed: float = 1.0) -> None:
        self.wake()
        self.ph.off()
        self.en.value = self._clamp_speed(speed)

    def reverse(self, speed: float = 1.0) -> None:
        self.wake()
        self.ph.on()
        self.en.value = self._clamp_speed(speed)

    def brake(self) -> None:
        self.wake()
        self.en.value = 0.0

    def sleep_mode(self) -> None:
        self.en.value = 0.0
        self.ph.off()
        self.sleep.off()

    def close(self) -> None:
        self.sleep_mode()
        self.en.close()
        self.ph.close()
        self.sleep.close()

    @staticmethod
    def _clamp_speed(speed: float) -> float:
        return max(0.0, min(1.0, speed))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control a DC motor with DRV8838.")
    parser.add_argument(
        "command",
        choices=("forward", "reverse", "brake", "sleep", "demo"),
        help="Motor command to run.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="PWM speed from 0.0 to 1.0. Used by forward/reverse/demo.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=2.0,
        help="Run time in seconds for forward/reverse/demo.",
    )
    parser.add_argument("--ph-pin", type=int, default=DEFAULT_PH_PIN)
    parser.add_argument("--en-pin", type=int, default=DEFAULT_EN_PIN)
    parser.add_argument("--sleep-pin", type=int, default=DEFAULT_SLEEP_PIN)
    parser.add_argument("--pwm-frequency", type=int, default=DEFAULT_PWM_FREQUENCY_HZ)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    motor = DRV8838Motor(
        ph_pin=args.ph_pin,
        en_pin=args.en_pin,
        sleep_pin=args.sleep_pin,
        pwm_frequency_hz=args.pwm_frequency,
    )

    signal.signal(signal.SIGTERM, lambda _signum, _frame: motor.close())

    try:
        if args.command == "forward":
            motor.forward(args.speed)
            time.sleep(args.seconds)
            motor.brake()
        elif args.command == "reverse":
            motor.reverse(args.speed)
            time.sleep(args.seconds)
            motor.brake()
        elif args.command == "brake":
            motor.brake()
        elif args.command == "sleep":
            motor.sleep_mode()
        elif args.command == "demo":
            motor.forward(args.speed)
            time.sleep(args.seconds)
            motor.brake()
            time.sleep(0.5)
            motor.reverse(args.speed)
            time.sleep(args.seconds)
            motor.brake()
    except KeyboardInterrupt:
        pass
    finally:
        motor.close()


if __name__ == "__main__":
    main()
