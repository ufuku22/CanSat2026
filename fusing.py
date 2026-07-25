import time

from gpiozero import OutputDevice

from config import FusingConfig


GPIO_PIN = 24


def fuse(seconds=FusingConfig.FUSE_DURATION_S):
    output = OutputDevice(GPIO_PIN, active_high=True, initial_value=False)
    try:
        output.on()
        time.sleep(seconds)
    finally:
        output.off()
        output.close()


def fuse_and_kick(
    driver,
    seconds=FusingConfig.FUSE_DURATION_S,
    speed=FusingConfig.KICK_SPEED,
    pulse_time=FusingConfig.KICK_PULSE_TIME_S,
):
    """溶断後、機体安定用にモーターを一瞬だけ後転させる。"""
    fuse(seconds)
    driver.flip(speed, pulse_time=pulse_time)
