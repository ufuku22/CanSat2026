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

    @staticmethod
    def _validate_speed(speed):
        """0から100までのデューティ比をfloatに変換する。"""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speedは0から100までの数値にしてください")
        if not 0 <= speed <= 100:
            raise ValueError("speedは0から100の範囲にしてください")
        return float(speed)

    @staticmethod
    def _validate_drive_speed(speed):
        """-100から100までの走行速度をfloatに変換する。"""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speedは-100から100までの数値にしてください")
        if not -100 <= speed <= 100:
            raise ValueError("speedは-100から100の範囲にしてください")
        return float(speed)

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("DriveControllerはすでにcleanup済みです")

    def _set_duty_cycle(self, speed):
        duty = speed / 100.0
        self.pwm_l.value = duty
        self.pwm_r.value = duty
        self._speed = speed

    def _set_duty_cycles(self, left_speed, right_speed):
        self.pwm_l.value = left_speed / 100.0
        self.pwm_r.value = right_speed / 100.0
        self._speed = max(left_speed, right_speed)

    def _disable_outputs(self):
        """TB6612FNGの出力を無効にする。"""
        self.stby.off()
        self._set_duty_cycle(0)

    def _prepare_motion(self, ain1, ain2, bin1, bin2):
        """方向切り替え時の衝撃を減らすため、出力を切ってから方向を設定する。"""
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
        """0%から目標速度まで少しずつ加速する。"""
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

        print(f"DriveController: {action}（目標速度: {speed:g}%）")
        self._prepare_motion(ain1, ain2, bin1, bin2)
        self._soft_start(speed)

    def drive(self, speed):
        """直進する。正の値は前進、負の値は後退、0は停止。"""
        self._ensure_open()
        speed = self._validate_drive_speed(speed)
        if speed >= 0:
            self._move("前進", speed, True, False, True, False)
        else:
            self._move("後退", abs(speed), False, True, False, True)

    def turn_right(self, speed):
        """その場で右旋回する。"""
        self._move("右旋回", speed, True, False, False, True)

    def turn_left(self, speed):
        """その場で左旋回する。"""
        self._move("左旋回", speed, False, True, True, False)

    def forward_differential(self, left_speed, right_speed):
        """左右のデューティ比を個別に指定して前進する。"""
        self._ensure_open()
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)

        if left_speed == 0 and right_speed == 0:
            self.stop()
            return

        self.ain1.value = True
        self.ain2.value = False
        self.bin1.value = True
        self.bin2.value = False
        self.stby.on()
        self._set_duty_cycles(left_speed, right_speed)

    def ramp_stop_forward(self, left_speed, right_speed, steps=100, interval=0.03):
        """前進中の左右デューティ比を少しずつ下げて停止する。"""
        steps = max(1, int(steps))
        left_speed = self._validate_speed(left_speed)
        right_speed = self._validate_speed(right_speed)

        for step in range(steps - 1, -1, -1):
            ratio = step / steps
            self.forward_differential(left_speed * ratio, right_speed * ratio)
            time.sleep(interval)
        self.stop()

    def reverse_stabilizer(self, speed, pulse_time=0.1):
        """スタビライザー反転用に、指定出力を一瞬だけ逆方向へ入れて停止する。"""
        self._ensure_open()
        speed = self._validate_speed(speed)
        if isinstance(pulse_time, bool) or not isinstance(pulse_time, numbers.Real):
            raise TypeError("pulse_timeは秒数を表す数値にしてください")
        if pulse_time <= 0:
            raise ValueError("pulse_timeは0より大きい値にしてください")

        print(f"DriveController: スタビライザー反転（出力: {speed:g}%, 時間: {pulse_time:g}秒）")
        self._prepare_motion(False, True, False, True)
        self._set_duty_cycle(speed)
        try:
            time.sleep(float(pulse_time))
        finally:
            self.stop()

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
