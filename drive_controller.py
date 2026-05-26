import numbers
import time

import RPi.GPIO as GPIO


class DriveController:
    """TB6612FNG を使用して左右の DC モーターを制御する。"""

    PWM_FREQUENCY_HZ = 100
    RAMP_STEP_PERCENT = 5.0
    RAMP_INTERVAL_S = 0.03
    DIRECTION_CHANGE_DELAY_S = 0.1

    def __init__(self):
        # BCM 番号。PWMA/PWMB は Raspberry Pi の PWM 対応ピン。
        self.PIN_STBY = 21
        self.PIN_PWMA = 12
        self.PIN_AIN1 = 23
        self.PIN_AIN2 = 18
        self.PIN_PWMB = 19
        self.PIN_BIN1 = 16
        self.PIN_BIN2 = 26

        self.pwm_l = None
        self.pwm_r = None
        self._speed = 0.0
        self._closed = False
        self._setup()

    def _setup(self):
        """ドライバを無効状態のまま初期化する。"""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.PIN_STBY, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(
            [self.PIN_AIN1, self.PIN_AIN2, self.PIN_BIN1, self.PIN_BIN2],
            GPIO.OUT,
            initial=GPIO.LOW,
        )
        GPIO.setup([self.PIN_PWMA, self.PIN_PWMB], GPIO.OUT, initial=GPIO.LOW)

        self.pwm_l = GPIO.PWM(self.PIN_PWMA, self.PWM_FREQUENCY_HZ)
        self.pwm_r = GPIO.PWM(self.PIN_PWMB, self.PWM_FREQUENCY_HZ)
        self.pwm_l.start(0)
        self.pwm_r.start(0)
        print("DriveController: 初期化完了")

    @staticmethod
    def _validate_speed(speed):
        """PWM duty cycle として使える 0 から 100 の数値へ変換する。"""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speed は 0 から 100 の数値で指定してください")
        if not 0 <= speed <= 100:
            raise ValueError("speed は 0 から 100 の範囲で指定してください")
        return float(speed)

    @staticmethod
    def _validate_drive_speed(speed):
        """前後移動用の -100 から 100 の速度値へ変換する。"""
        if isinstance(speed, bool) or not isinstance(speed, numbers.Real):
            raise TypeError("speed は -100 から 100 の数値で指定してください")
        if not -100 <= speed <= 100:
            raise ValueError("speed は -100 から 100 の範囲で指定してください")
        return float(speed)

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("DriveController は cleanup 済みです")

    def _set_duty_cycle(self, speed):
        self.pwm_l.ChangeDutyCycle(speed)
        self.pwm_r.ChangeDutyCycle(speed)
        self._speed = speed

    def _disable_outputs(self):
        """TB6612FNG をスタンバイにし、モーター出力をハイインピーダンスにする。"""
        GPIO.output(self.PIN_STBY, GPIO.LOW)
        self._set_duty_cycle(0)

    def _prepare_motion(self, ain1, ain2, bin1, bin2):
        """出力を切った状態で方向を変更し、逆転衝撃を小さくする。"""
        was_moving = self._speed > 0
        self._disable_outputs()
        if was_moving:
            time.sleep(self.DIRECTION_CHANGE_DELAY_S)

        GPIO.output(self.PIN_AIN1, ain1)
        GPIO.output(self.PIN_AIN2, ain2)
        GPIO.output(self.PIN_BIN1, bin1)
        GPIO.output(self.PIN_BIN2, bin2)
        GPIO.output(self.PIN_STBY, GPIO.HIGH)

    def _soft_start(self, target_speed):
        """0% から指定速度まで段階的に加速する。"""
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

        print(f"DriveController: {action}します (目標速度: {speed:g}%)")
        self._prepare_motion(ain1, ain2, bin1, bin2)
        self._soft_start(speed)

    def drive(self, speed):
        """符号付き速度で直進する。正値は前進、負値は後退、0 は停止。"""
        self._ensure_open()
        speed = self._validate_drive_speed(speed)
        if speed >= 0:
            self._move("前進", speed, GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
        else:
            self._move("後退", abs(speed), GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)

    def turn_right(self, speed):
        """その場で右旋回する。"""
        self._move("右旋回", speed, GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)

    def turn_left(self, speed):
        """その場で左旋回する。"""
        self._move("左旋回", speed, GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW)

    def stop(self):
        """出力を無効化して、惰性で停止させる。"""
        self._ensure_open()
        print("DriveController: 惰性停止します")
        self._disable_outputs()
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)

    def brake(self):
        """TB6612FNG のショートブレーキで急制動する。"""
        self._ensure_open()
        print("DriveController: 急制動します")
        self._set_duty_cycle(0)
        GPIO.output(self.PIN_STBY, GPIO.HIGH)
        self._speed = 0.0

    def cleanup(self):
        """モーター出力を無効化し、GPIO を解放する。"""
        if self._closed:
            return

        self.stop()
        self.pwm_l.stop()
        self.pwm_r.stop()
        GPIO.output(self.PIN_STBY, GPIO.LOW)
        GPIO.cleanup()
        self._closed = True
        print("DriveController: GPIO を解放しました")


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
        print("強制終了")
    finally:
        driver.cleanup()
