import time

from gpiozero import OutputDevice, PWMOutputDevice

from config import DriveControllerConfig as DriveConfig


class DriveController(DriveConfig):
    """TB6612FNGを使って左右のDCモーターを制御する。"""

    PIN_STBY = 21
    PIN_PWMA = 12
    PIN_AIN1 = 8
    PIN_AIN2 = 7
    PIN_PWMB = 19
    PIN_BIN1 = 25
    PIN_BIN2 = 26

    def __init__(self):
        self.stby = None
        self.ain1 = None
        self.ain2 = None
        self.bin1 = None
        self.bin2 = None
        self.pwm_l = None
        self.pwm_r = None
        self._speed = 0.0
        self._left_speed = 0.0
        self._right_speed = 0.0
        self._closed = False
        self.invert_left_motor = self.INVERT_LEFT_MOTOR
        self.invert_right_motor = self.INVERT_RIGHT_MOTOR
        self.left_motor_gain = self.LEFT_MOTOR_GAIN
        self.right_motor_gain = self.RIGHT_MOTOR_GAIN
        self._setup()

    def _setup(self):
        """出力を無効にした状態でドライバを初期化する。"""
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

    def set_motor_inversion(self, invert_left_motor=None, invert_right_motor=None):
        """左右モーターの回転方向反転設定を変更する。"""
        if invert_left_motor is not None:
            self.invert_left_motor = bool(invert_left_motor)
        if invert_right_motor is not None:
            self.invert_right_motor = bool(invert_right_motor)

    def set_motor_gain(self, left_motor_gain=None, right_motor_gain=None):
        """左右モーターの出力補正倍率を変更する。"""
        if left_motor_gain is not None:
            self.left_motor_gain = max(0.0, float(left_motor_gain))
        if right_motor_gain is not None:
            self.right_motor_gain = max(0.0, float(right_motor_gain))

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("DriveControllerはすでにcleanup済みです")

    def _set_duty_cycle(self, speed):
        self._set_duty_cycles(speed, speed)

    def _set_duty_cycles(self, left_speed, right_speed):
        left_speed = max(0.0, min(float(left_speed), 100.0))
        right_speed = max(0.0, min(float(right_speed), 100.0))
        corrected_left = max(
            0.0,
            min(left_speed * self.left_motor_gain, 100.0),
        )
        corrected_right = max(
            0.0,
            min(right_speed * self.right_motor_gain, 100.0),
        )
        self.pwm_l.value = corrected_left / 100.0
        self.pwm_r.value = corrected_right / 100.0
        self._left_speed = left_speed
        self._right_speed = right_speed
        self._speed = max(corrected_left, corrected_right)

    def _disable_outputs(self):
        """TB6612FNGの出力を無効にする。"""
        self.stby.off()
        self._set_duty_cycle(0)

    def _motor_direction_pins(self, forward, inverted):
        logical_forward = bool(forward)
        if inverted:
            logical_forward = not logical_forward
        return logical_forward, not logical_forward

    def _prepare_motion(self, left_forward, right_forward):
        """方向切り替え時の衝撃を減らすため、出力を切ってから方向を設定する。"""
        was_moving = self._speed > 0
        self._disable_outputs()
        if was_moving:
            time.sleep(self.DIRECTION_CHANGE_DELAY_S)

        ain1, ain2 = self._motor_direction_pins(left_forward, self.invert_left_motor)
        bin1, bin2 = self._motor_direction_pins(right_forward, self.invert_right_motor)
        self.ain1.value = ain1
        self.ain2.value = ain2
        self.bin1.value = bin1
        self.bin2.value = bin2
        self.stby.on()

    def _soft_start(self, target_speed):
        """0%から目標速度まで少しずつ加速する。"""
        speed = 0.0
        while speed < target_speed:
            speed = min(speed + self.SOFT_START_STEP_PERCENT, target_speed)
            self._set_duty_cycle(speed)
            time.sleep(self.SOFT_START_INTERVAL_S)

    def _move(self, action, speed, left_forward, right_forward):
        self._ensure_open()
        speed = max(0.0, min(float(speed), 100.0))
        if speed == 0:
            self.stop()
            return

        self._prepare_motion(left_forward, right_forward)
        self._soft_start(speed)

    def drive(self, speed):
        """直進する。正の値は前進、負の値は後退、0は停止。"""
        self._ensure_open()
        speed = max(-100.0, min(float(speed), 100.0))
        if speed >= 0:
            self._move("前進", speed, True, True)
        else:
            self._move("後退", abs(speed), False, False)

    def turn_right(self, speed):
        """その場で右旋回する。"""
        self._move("右旋回", speed, True, False)

    def turn_left(self, speed):
        """その場で左旋回する。"""
        self._move("左旋回", speed, False, True)

    def forward_differential(self, left_speed, right_speed):
        """左右のデューティ比を個別に指定して前進する。"""
        self._ensure_open()
        left_speed = max(0.0, min(float(left_speed), 100.0))
        right_speed = max(0.0, min(float(right_speed), 100.0))

        if left_speed == 0 and right_speed == 0:
            self.stop()
            return

        ain1, ain2 = self._motor_direction_pins(True, self.invert_left_motor)
        bin1, bin2 = self._motor_direction_pins(True, self.invert_right_motor)
        self.ain1.value = ain1
        self.ain2.value = ain2
        self.bin1.value = bin1
        self.bin2.value = bin2
        self.stby.on()
        self._set_duty_cycles(left_speed, right_speed)

    def get_forward_motor_outputs(self):
        """前進指示中の左右モーター出力を返し、それ以外ではNoneを返す。"""
        left_forward = self._motor_direction_pins(True, self.invert_left_motor)
        right_forward = self._motor_direction_pins(True, self.invert_right_motor)
        if (
            not self.stby.value
            or (self.ain1.value, self.ain2.value) != left_forward
            or (self.bin1.value, self.bin2.value) != right_forward
        ):
            return None
        return self._left_speed, self._right_speed

    def ramp_stop_forward(
        self,
        left_speed,
        right_speed,
        steps=DriveConfig.RAMP_STOP_STEPS,
        interval=DriveConfig.RAMP_STOP_INTERVAL_S,
    ):
        """前進中の左右デューティ比を少しずつ下げて停止する。"""
        steps = max(1, int(steps))
        left_speed = max(0.0, min(float(left_speed), 100.0))
        right_speed = max(0.0, min(float(right_speed), 100.0))

        for step in range(steps - 1, -1, -1):
            ratio = step / steps
            self.forward_differential(left_speed * ratio, right_speed * ratio)
            time.sleep(interval)
        self.stop()

    def ramp_stop_current_forward(
        self,
        steps=DriveConfig.RAMP_STOP_STEPS,
        interval=DriveConfig.RAMP_STOP_INTERVAL_S,
    ):
        """現在の左右前進出力を基準に、少しずつ下げて停止する。"""
        self.ramp_stop_forward(
            self._left_speed,
            self._right_speed,
            steps=steps,
            interval=interval,
        )

    def reverse_stabilizer(
        self,
        speed=DriveConfig.STABILIZER_SPEED,
        pulse_time=DriveConfig.STABILIZER_PULSE_TIME_S,
    ):
        """ひっくり返った機体を元に戻す"""
        speed = max(0.0, min(float(speed), 100.0))
        pulse_time = float(pulse_time)

        self._prepare_motion(True, True)
        self._set_duty_cycle(speed)
        try:
            time.sleep(pulse_time)
        finally:
            self.brake()
    
    def flip(
        self,
        speed=DriveConfig.STABILIZER_SPEED,
        pulse_time=DriveConfig.STABILIZER_PULSE_TIME_S,
    ):
        """機体をひっくり返す"""
        speed = max(0.0, min(float(speed), 100.0))
        pulse_time = float(pulse_time)

        self._prepare_motion(False, False)
        self._set_duty_cycle(speed)
        try:
            time.sleep(pulse_time)
        finally:
            self.brake()

    def stop(self):
        """出力を切って慣性で停止する。"""
        self._ensure_open()
        self._disable_outputs()
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()

    def brake(self):
        """両モーターを短絡ブレーキする。"""
        self._ensure_open()
        self._set_duty_cycle(0)
        self.stby.on()
        self._speed = 0.0

    def cleanup(self):
        """モーター出力を止め、このコントローラが使っているピンだけを解放する。"""
        if self._closed:
            return

        self.stop()
        for device in (self.pwm_l, self.pwm_r, self.stby, self.ain1, self.ain2, self.bin1, self.bin2):
            device.close()
        self._closed = True


if __name__ == "__main__":
    driver = DriveController()
    try:
        driver.drive(100)
        time.sleep(5)
        driver.stop()
        time.sleep(1)

        driver.drive(-100)
        time.sleep(5)
        driver.stop()
        time.sleep(1)

        driver.turn_right(100)
        time.sleep(5)
        driver.stop()
    except KeyboardInterrupt:
        print("中断しました")
    finally:
        driver.cleanup()
