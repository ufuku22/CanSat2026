import numbers
import time

from gpiozero import OutputDevice, PWMOutputDevice


class DriveController:
    """TB6612FNGを使って左右のDCモーターを制御する。"""

    PWM_FREQUENCY_HZ = 100
    RAMP_STEP_PERCENT = 5.0
    RAMP_INTERVAL_S = 0.03
    DIRECTION_CHANGE_DELAY_S = 0.1

    def __init__(self):
        # GPIOの番号
        self.PIN_STBY = 21
        self.PIN_PWMA = 12
        self.PIN_AIN1 = 8
        self.PIN_AIN2 = 7
        self.PIN_PWMB = 19
        self.PIN_BIN1 = 25
        self.PIN_BIN2 = 26

        self.stby = None
        self.ain1 = None
        self.ain2 = None
        self.bin1 = None
        self.bin2 = None
        self.pwm_l = None
        self.pwm_r = None
        self._speed = 0.0
        self._closed = False
        self._setup()

    def _setup(self):
        """Initialize the driver with outputs disabled."""
        self.stby = OutputDevice(self.PIN_STBY, active_high=True, initial_value=False)
        self.ain1 = OutputDevice(self.PIN_AIN1, active_high=True, initial_value=False)
        self.ain2 = OutputDevice(self.PIN_AIN2, active_high=True, initial_value=False)
        self.bin1 = OutputDevice(self.PIN_BIN1, active_high=True, initial_value=False)
        self.bin2 = OutputDevice(self.PIN_BIN2, active_high=True, initial_value=False)
        self.pwm_l = PWMOutputDevice(
            self.PIN_PWMA,
            active_high=True,
            initial_value=0.0,
            frequency=self.PWM_FREQUENCY_HZ,
        )
        self.pwm_r = PWMOutputDevice(
            self.PIN_PWMB,
            active_high=True,
            initial_value=0.0,
            frequency=self.PWM_FREQUENCY_HZ,
        )
        print("DriveController: initialized")

    @staticmethod
    def _validate_speed(speed):
        """Convert a 0-100 duty cycle value to float."""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speed must be a number from 0 to 100")
        if not 0 <= speed <= 100:
            raise ValueError("speed must be in the range 0 to 100")
        return float(speed)

    @staticmethod
    def _validate_drive_speed(speed):
        """Convert a -100 to 100 drive speed value to float."""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speed must be a number from -100 to 100")
        if not -100 <= speed <= 100:
            raise ValueError("speed must be in the range -100 to 100")
        return float(speed)

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("DriveController has already been cleaned up")

    def _set_duty_cycle(self, speed):
        duty = speed / 100.0
        self.pwm_l.value = duty
        self.pwm_r.value = duty
        self._speed = speed

    def _disable_outputs(self):
        """Disable TB6612FNG outputs."""
        self.stby.off()
        self._set_duty_cycle(0)

    def _prepare_motion(self, ain1, ain2, bin1, bin2):
        """Set direction while output is disabled to reduce direction-change shock."""
        was_moving = self._speed > 0
        self._disable_outputs()
        if was_moving:
            time.sleep(self.DIRECTION_CHANGE_DELAY_S)

        self.ain1.value = ain1
        self.ain2.value = ain2
        self.bin1.value = bin1
        self.bin2.value = bin2
        self.stby.on()

    def _soft_start(self, target_speed):
        """Ramp up from 0% to the target speed."""
        speed = 0.0
        while speed < target_speed:
            speed = min(speed + self.RAMP_STEP_PERCENT, target_speed)
            self._set_duty_cycle(speed)
            time.sleep(self.RAMP_INTERVAL_S)

    def _move(self, action, speed, ain1, ain2, bin1, bin2):
        self._ensure_open()
        speed = self._validate_speed(speed)
        if speed == 0:
            self.stop()
            return

        print(f"DriveController: {action} (target speed: {speed:g}%)")
        self._prepare_motion(ain1, ain2, bin1, bin2)
        self._soft_start(speed)

    def drive(self, speed):
        """Drive straight. Positive is forward, negative is reverse, and 0 stops."""
        self._ensure_open()
        speed = self._validate_drive_speed(speed)
        if speed >= 0:
            self._move("forward", speed, True, False, True, False)
        else:
            self._move("reverse", abs(speed), False, True, False, True)

    def turn_right(self, speed):
        """Turn right in place."""
        self._move("turn right", speed, True, False, False, True)

    def turn_left(self, speed):
        """Turn left in place."""
        self._move("turn left", speed, False, True, True, False)

    def stop(self):
        """Disable outputs and stop by inertia."""
        self._ensure_open()
        print("DriveController: stop")
        self._disable_outputs()
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()

    def brake(self):
        """Short brake both motors."""
        self._ensure_open()
        print("DriveController: brake")
        self._set_duty_cycle(0)
        self.stby.on()
        self._speed = 0.0

    def cleanup(self):
        """Disable motor outputs and release only the pins owned by this controller."""
        if self._closed:
            return

        self.stop()
        for device in (self.pwm_l, self.pwm_r, self.stby, self.ain1, self.ain2, self.bin1, self.bin2):
            device.close()
        self._closed = True
        print("DriveController: GPIO devices closed")


if __name__ == "__main__":
    driver = DriveController()
    try:
        driver.drive(60)
        time.sleep(2)
        driver.stop()
        time.sleep(1)

        driver.drive(-40)
        time.sleep(2)
        driver.stop()
        time.sleep(1)

        driver.turn_right(50)
        time.sleep(2)
        driver.stop()
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        driver.cleanup()
