from time import sleep

from gpiozero import PWMOutputDevice, OutputDevice


# BCM GPIO numbers.
AIN1 = 17
AIN2 = 27
PWMA = 18
STBY = 9


def wait(message):
    input(f"\n{message}\nPress Enter to continue...")


def set_outputs(standby, ain1, ain2, pwm, stby, a1, a2, speed):
    standby.value = stby
    ain1.value = a1
    ain2.value = a2
    pwm.value = speed
    print(
        f"STBY={stby} AIN1={a1} AIN2={a2} PWMA={speed:.2f} "
        f"(GPIO {STBY}, {AIN1}, {AIN2}, {PWMA})"
    )


def main():
    standby = OutputDevice(STBY, initial_value=False)
    ain1 = OutputDevice(AIN1, initial_value=False)
    ain2 = OutputDevice(AIN2, initial_value=False)
    pwm = PWMOutputDevice(PWMA, frequency=1000, initial_value=0)

    try:
        print("TB6612FNG diagnostic start")
        print("Use a tester if available: GND reference should be common with Raspberry Pi GND.")

        wait("1) Standby OFF. Motor should not move. STBY should be about 0V.")
        set_outputs(standby, ain1, ain2, pwm, stby=0, a1=1, a2=0, speed=0.6)
        sleep(3)

        wait("2) Standby ON, forward. STBY/AIN1 should be about 3.3V, AIN2 about 0V.")
        set_outputs(standby, ain1, ain2, pwm, stby=1, a1=1, a2=0, speed=0.6)
        sleep(3)

        wait("3) Reverse. AIN1 should be about 0V, AIN2 about 3.3V.")
        set_outputs(standby, ain1, ain2, pwm, stby=1, a1=0, a2=1, speed=0.6)
        sleep(3)

        wait("4) Brake/stop. AIN1/AIN2 both 0V, PWM still present, motor should stop.")
        set_outputs(standby, ain1, ain2, pwm, stby=1, a1=0, a2=0, speed=0.6)
        sleep(2)

        wait("5) PWM ramp forward. Motor speed should gradually increase.")
        set_outputs(standby, ain1, ain2, pwm, stby=1, a1=1, a2=0, speed=0.0)
        for duty in (0.2, 0.4, 0.6, 0.8, 1.0):
            pwm.value = duty
            print(f"PWMA duty={duty:.1f}")
            sleep(2)

        print("\nDone.")
    finally:
        pwm.value = 0
        ain1.off()
        ain2.off()
        standby.off()
        pwm.close()
        ain1.close()
        ain2.close()
        standby.close()


if __name__ == "__main__":
    main()
