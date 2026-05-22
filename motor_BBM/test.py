import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

# 正転（2秒）
GPIO.output(17, 1)
GPIO.output(27, 0)
time.sleep(2)

# 停止
GPIO.output(17, 0)
GPIO.output(27, 0)

GPIO.cleanup()
