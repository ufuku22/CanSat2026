import RPi.GPIO as GPIO
import time

class DriveController:
    """
    左右のモータを安全に制御し、機体の走行を行う最下層クラス（ソフトスタート機能付き）
    """
    def __init__(self):
        # --- ピン設定 (BCM番号) ---
        self.PIN_STBY = 21
        self.PIN_PWMA = 12
        self.PIN_AIN1 = 23
        self.PIN_AIN2 = 18
        self.PIN_PWMB = 19
        self.PIN_BIN1 = 16
        self.PIN_BIN2 = 26
        
        self._setup()

    def _setup(self):
        """GPIOピンの初期設定"""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        pins = [self.PIN_STBY, 
                self.PIN_PWMA, self.PIN_AIN1, self.PIN_AIN2,
                self.PIN_PWMB, self.PIN_BIN1, self.PIN_BIN2]
        GPIO.setup(pins, GPIO.OUT)
        GPIO.output(self.PIN_STBY, GPIO.HIGH)
        
        # PWM設定 (100Hz)
        self.pwm_l = GPIO.PWM(self.PIN_PWMA, 100)
        self.pwm_r = GPIO.PWM(self.PIN_PWMB, 100)
        self.pwm_l.start(0)
        self.pwm_r.start(0)
        print("DriveController: 初期化完了（全動作セーフティモード有効）")

    def _soft_start(self, target_speed):
        """【安全装置】電流スパイクを防ぐため、0%から目標速度までじわじわ加速する"""
        # 0.03秒ごとに5%ずつ加速（約0.5秒で目標速度に達するスムーズ設計）
        for speed in range(0, target_speed + 1, 5):
            self.pwm_l.ChangeDutyCycle(speed)
            self.pwm_r.ChangeDutyCycle(speed)
            time.sleep(0.03)

    def forward(self, speed):
        """安全に前進する"""
        print(f"DriveController: 前進します (目標速度: {speed}%)")
        GPIO.output(self.PIN_AIN1, GPIO.HIGH)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        GPIO.output(self.PIN_BIN1, GPIO.HIGH)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        self._soft_start(speed)

    def backward(self, speed):
        """安全に後退する"""
        print(f"DriveController: 後退します (目標速度: {speed}%)")
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.HIGH)
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.HIGH)
        self._soft_start(speed)

    def turn_right(self, speed):
        """安全に右旋回する"""
        print(f"DriveController: 右旋回します (目標速度: {speed}%)")
        GPIO.output(self.PIN_AIN1, GPIO.HIGH)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.HIGH)
        self._soft_start(speed)

    def turn_left(self, speed):
        """安全に左旋回する"""
        print(f"DriveController: 左旋回します (目標速度: {speed}%)")
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.HIGH)
        GPIO.output(self.PIN_BIN1, GPIO.HIGH)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        self._soft_start(speed)

    def stop(self):
        """モーターを停止する（停止時は電流スパイクが起きないため一瞬でOK）"""
        print("DriveController: 停止します")
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0)

    def cleanup(self):
        """GPIOの解放"""
        self.stop()
        if hasattr(self, 'pwm_l'): self.pwm_l.stop()
        if hasattr(self, 'pwm_r'): self.pwm_r.stop()
        GPIO.cleanup()
        print("DriveController: GPIOを解放しました")

# =====================================================================
# 🔬 テスト用プログラム
# =====================================================================
if __name__ == "__main__":
    driver = DriveController()
    try:
        driver.forward(60)
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