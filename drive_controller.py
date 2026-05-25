import RPi.GPIO as GPIO
import time

class DriveController:
    """
    左右のモータ（TB6612）を制御し、機体の走行を行うクラス
    """
    def __init__(self):
        # --- 回路図に基づくピン設定 (BCM番号) ---
        # 共通
        self.PIN_STBY = 12
        
        # 左モーター (Channel A)
        self.PIN_PWMA = 19
        self.PIN_AIN1 = 26
        self.PIN_AIN2 = 16
        
        # 右モーター (Channel B)
        self.PIN_PWMB = 13
        self.PIN_BIN1 = 20
        self.PIN_BIN2 = 21
        
        # 初期化関数の呼び出し
        self._setup()

    def _setup(self):
        """GPIOピンの初期設定を行う（内部用）"""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        # 使用する全ピンを出力モードに設定
        pins = [self.PIN_STBY, 
                self.PIN_PWMA, self.PIN_AIN1, self.PIN_AIN2,
                self.PIN_PWMB, self.PIN_BIN1, self.PIN_BIN2]
        GPIO.setup(pins, GPIO.OUT)
        
        # STBYピンをHIGHにしてモータードライバを叩き起こす
        GPIO.output(self.PIN_STBY, GPIO.HIGH)
        
        # PWMの設定 (周波数100Hz)
        self.pwm_l = GPIO.PWM(self.PIN_PWMA, 100)
        self.pwm_r = GPIO.PWM(self.PIN_PWMB, 100)
        self.pwm_l.start(0)
        self.pwm_r.start(0)
        print("DriveController: 初期化が完了しました（左右モータ設定OK）")

    def forward(self, speed):
        """左右のモータを正転させて前進する"""
        print(f"DriveController: まっすぐ前進します (速度: {speed}%)")
        # 左モーター正転
        GPIO.output(self.PIN_AIN1, GPIO.HIGH)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        # 右モーター正転
        GPIO.output(self.PIN_BIN1, GPIO.HIGH)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        
        # スピード適用
        self.pwm_l.ChangeDutyCycle(speed)
        self.pwm_r.ChangeDutyCycle(speed)

    def backward(self, speed):
        """左右のモータを逆転させて後退する"""
        print(f"DriveController: まっすぐ後退します (速度: {speed}%)")
        # 左モーター逆転
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.HIGH)
        # 右モーター逆転
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.HIGH)
        
        # スピード適用
        self.pwm_l.ChangeDutyCycle(speed)
        self.pwm_r.ChangeDutyCycle(speed)

    def stop(self):
        """左右のモータを停止させる"""
        print("DriveController: 停止します")
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(0)

    def turn_right(self, speed):
        """右にその場で旋回する"""
        print(f"DriveController: 右に旋回します (速度: {speed}%)")
        # 左モーターは正転（前に進む）
        GPIO.output(self.PIN_AIN1, GPIO.HIGH)
        GPIO.output(self.PIN_AIN2, GPIO.LOW)
        # 右モーターは逆転（後ろに下がる）
        GPIO.output(self.PIN_BIN1, GPIO.LOW)
        GPIO.output(self.PIN_BIN2, GPIO.HIGH)
        
        # スピード適用
        self.pwm_l.ChangeDutyCycle(speed)
        self.pwm_r.ChangeDutyCycle(speed)

    def turn_left(self, speed):
        """左にその場で旋回する"""
        print(f"DriveController: 左に旋回します (速度: {speed}%)")
        # 左モーターは逆転（後ろに下がる）
        GPIO.output(self.PIN_AIN1, GPIO.LOW)
        GPIO.output(self.PIN_AIN2, GPIO.HIGH)
        # 右モーターは正転（前に進む）
        GPIO.output(self.PIN_BIN1, GPIO.HIGH)
        GPIO.output(self.PIN_BIN2, GPIO.LOW)
        
        # スピード適用
        self.pwm_l.ChangeDutyCycle(speed)
        self.pwm_r.ChangeDutyCycle(speed)
        
    def cleanup(self):
        """終了時にGPIOを安全に解放する"""
        self.stop()
        if hasattr(self, 'pwm_l'): self.pwm_l.stop()
        if hasattr(self, 'pwm_r'): self.pwm_r.stop()
        GPIO.cleanup()
        print("DriveController: GPIOを解放しました")

# =====================================================================
# 🔬 テスト用プログラム（ラズパイで直接このファイルを実行した時だけ動く）
# =====================================================================
if __name__ == "__main__":
    driver = DriveController()
    
    try:
        # 50%の速度で3秒間前進
        driver.forward(50)
        time.sleep(3)
        
        # 1秒停止
        driver.stop()
        time.sleep(1)
        
        # 50%の速度で3秒間後退
        driver.backward(50)
        time.sleep(3)

    except KeyboardInterrupt:
        print("プログラムを強制終了します")
        
    finally:
        driver.cleanup()