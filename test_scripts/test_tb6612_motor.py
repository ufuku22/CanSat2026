from time import sleep

from gpiozero import Motor, OutputDevice


# BCM GPIO numbers. Change these to match your wiring.
AIN1 = 17
AIN2 = 27
PWMA = 18
STBY = 9

TEST_SPEED = 0.45
RUN_SECONDS = 2.0


def main():
    standby = OutputDevice(STBY, initial_value=False)
    motor = Motor(forward=AIN1, backward=AIN2, enable=PWMA, pwm=True)

    try:
        standby.on()

        print("forward")
        motor.forward(TEST_SPEED)
        sleep(RUN_SECONDS)

        print("stop")
        motor.stop()
        sleep(1.0)

        print("backward")
        motor.backward(TEST_SPEED)
        sleep(RUN_SECONDS)

        print("stop")
        motor.stop()
    finally:
        motor.stop()
        standby.off()


if __name__ == "__main__":
    main()
