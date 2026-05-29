#!/usr/bin/env python3
"""DRV8838 motor test: forward 10 seconds, then reverse 10 seconds."""

import time

from gpiozero import OutputDevice, PWMOutputDevice


# BCM GPIO numbers. Change these if your wiring is different.
PH_PIN = 5
EN_PIN = 13
SLEEP_PIN = 6

SPEED = 0.6
RUN_SECONDS = 30
STOP_SECONDS = 10
PWM_FREQUENCY_HZ = 1000


ph = OutputDevice(PH_PIN, active_high=True, initial_value=False)
en = PWMOutputDevice(
    EN_PIN,
    active_high=True,
    initial_value=0.0,
    frequency=PWM_FREQUENCY_HZ,
)
sleep = OutputDevice(SLEEP_PIN, active_high=True, initial_value=False)


def wake_driver():
    sleep.on()
    time.sleep(0.002)


def forward():
    wake_driver()
    ph.off()
    en.value = SPEED


def reverse():
    wake_driver()
    ph.on()
    en.value = SPEED


def brake():
    wake_driver()
    en.value = 0.0


def forward_10_seconds():
    print("Forward 10 seconds")
    forward()
    time.sleep(RUN_SECONDS)


def stop_10_seconds():
    print("Stop 10 seconds")
    brake()
    time.sleep(STOP_SECONDS)


def sleep_driver():
    en.value = 0.0
    ph.off()
    sleep.off()


try:
    forward_10_seconds()
    stop_10_seconds()

    print("Reverse 10 seconds")
    reverse()
    time.sleep(RUN_SECONDS)

    print("Brake")
    brake()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    sleep_driver()
    en.close()
    ph.close()
    sleep.close()
