import time

from gpiozero import OutputDevice, PWMOutputDevice


class DriveController:
    """TB6612FNGを使って左右のDCモーターを制御する。"""

    PWM_FREQUENCY_HZ = 100
    RAMP_STEP_PERCENT = 5.0
    RAMP_INTERVAL_S = 0.03
    DIRECTION_CHANGE_DELAY_S = 0.1
    DEFAULT_INVERT_LEFT_MOTOR = True      #タイヤの回転方向を反転したいときはここをTrueにする
    DEFAULT_INVERT_RIGHT_MOTOR = False
    DEFAULT_LEFT_MOTOR_GAIN = 1.0         # 左モーター出力補正。手打ちで0.95などに変更する
    DEFAULT_RIGHT_MOTOR_GAIN = 1.0        # 右モーター出力補正。初期値は補正なし

    def __init__(self):
        # GPIOのピン番号
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
        self.invert_left_motor = self.DEFAULT_INVERT_LEFT_MOTOR
        self.invert_right_motor = self.DEFAULT_INVERT_RIGHT_MOTOR
        self.left_motor_gain = self.DEFAULT_LEFT_MOTOR_GAIN
        self.right_motor_gain = self.DEFAULT_RIGHT_MOTOR_GAIN
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
        print("DriveController: 初期化しました")

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
        corrected_left = max(0.0, min(float(left_speed) * self.left_motor_gain, 100.0))
        corrected_right = max(0.0, min(float(right_speed) * self.right_motor_gain, 100.0))
        self.pwm_l.value = corrected_left / 100.0
        self.pwm_r.value = corrected_right / 100.0
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
            speed = min(speed + self.RAMP_STEP_PERCENT, target_speed)
            self._set_duty_cycle(speed)
            time.sleep(self.RAMP_INTERVAL_S)

    def _move(self, action, speed, left_forward, right_forward):
        self._ensure_open()
        speed = max(0.0, min(float(speed), 100.0))
        if speed == 0:
            self.stop()
            return

        print(f"DriveController: {action}（目標速度: {speed:g}%）")
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

    def ramp_stop_forward(self, left_speed, right_speed, steps=100, interval=0.03):
        """前進中の左右デューティ比を少しずつ下げて停止する。"""
        steps = max(1, int(steps))
        left_speed = max(0.0, min(float(left_speed), 100.0))
        right_speed = max(0.0, min(float(right_speed), 100.0))

        for step in range(steps - 1, -1, -1):
            ratio = step / steps
            self.forward_differential(left_speed * ratio, right_speed * ratio)
            time.sleep(interval)
        self.stop()

    def reverse_stabilizer(self, speed=100, pulse_time=0.5):
        """スタビライザー反転用に、指定出力を一瞬だけ逆方向へ入れて停止する。"""
        speed = max(0.0, min(float(speed), 100.0))
        pulse_time = float(pulse_time)

        print(f"DriveController: スタビライザー反転（出力: {speed:g}%, 時間: {pulse_time:g}秒）")
        self._prepare_motion(True, True)
        self._set_duty_cycle(speed)
        try:
            time.sleep(pulse_time)
        finally:
            self.brake()

    def stop(self):
        """出力を切って慣性で停止する。"""
        self._ensure_open()
        print("DriveController: 停止")
        self._disable_outputs()
        self.ain1.off()
        self.ain2.off()
        self.bin1.off()
        self.bin2.off()

    def brake(self):
        """両モーターを短絡ブレーキする。"""
        self._ensure_open()
        print("DriveController: ブレーキ")
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
        print("DriveController: GPIOデバイスを解放しました")


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
