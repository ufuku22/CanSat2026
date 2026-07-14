import time

from gpiozero import OutputDevice

from drive_controller import DriveController


GPIO_PIN = 24


def fuse(seconds=3):
    output = OutputDevice(GPIO_PIN, active_high=True, initial_value=False)
    try:
        output.on()
        time.sleep(seconds)
    finally:
        output.off()
        output.close()


def fuse_and_kick(seconds=3, speed=100, pulse_time=0.1):
    """溶断後、機体安定用にモーターを一瞬だけ後転させる。"""
    fuse(seconds)
    driver = DriveController()
    try:
        driver.reverse_stabilizer(speed, pulse_time=pulse_time)
    finally:
        driver.cleanup()
