import RPi.GPIO as GPIO


MOSFET_PIN = 24


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOSFET_PIN, GPIO.OUT, initial=GPIO.LOW)


def nichrome_on():
    GPIO.output(MOSFET_PIN, GPIO.HIGH)


def cleanup():
    GPIO.output(MOSFET_PIN, GPIO.LOW)
    GPIO.cleanup()
