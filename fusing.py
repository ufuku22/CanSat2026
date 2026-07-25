import time

from gpiozero import OutputDevice


def fuse(seconds=3.0):
    output = OutputDevice(24, active_high=True, initial_value=False)
    try:
        output.on()
        time.sleep(seconds)
    finally:
        output.off()
        output.close()


def fuse_and_kick(
    driver,
    seconds=3.0,
    speed=100.0,
    pulse_time=0.1,
):
    """溶断後、機体安定用にモーターを一瞬だけ後転させる。"""
    fuse(seconds)
    driver.flip(speed, pulse_time=pulse_time)
