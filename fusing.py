import time

from gpiozero import OutputDevice


GPIO_PIN = 24


def fuse(seconds=3):
    output = OutputDevice(GPIO_PIN, active_high=True, initial_value=False)
    try:
        output.on()
        time.sleep(seconds)
    finally:
        output.off()
        output.close()
