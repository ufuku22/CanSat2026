import RPi.GPIO as GPIO
import time

# --- ピンの設定 (BCM番号) ---
PIN_AIN1 = 17
PIN_AIN2 = 27
PIN_PWMA = 18
PIN_STBY = 22

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    
    # ピンを出力モードに設定
    GPIO.setup([PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_STBY], GPIO.OUT)
    
    # 【重要】STBYピンをHIGHにしてスタンバイモードを解除
    GPIO.output(PIN_STBY, GPIO.HIGH)
    
    # PWMの設定 (周波数100Hz)
    global pwm_a
    pwm_a = GPIO.PWM(PIN_PWMA, 100)
    pwm_a.start(0)  # 初期状態は速度0%（停止）

def motor_forward(speed):
    print(f"前進します (速度: {speed}%)")
    GPIO.output(PIN_AIN1, GPIO.HIGH)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)  # 速度を変更 (0〜100)

def motor_backward(speed):
    print(f"後退します (速度: {speed}%)")
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(speed)

def motor_stop():
    print("停止します")
    GPIO.output(PIN_AIN1, GPIO.LOW)
    GPIO.output(PIN_AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)

def main():
    try:
        setup()
        
        # 50%の速度で3秒間前進
        motor_forward(50)
        time.sleep(3)
        
        # 1秒停止
        motor_stop()
        time.sleep(1)
        
        # 100%の速度で3秒間後退
        motor_backward(100)
        time.sleep(3)

    except KeyboardInterrupt:
        print("プログラムを強制終了します")
        
    finally:
        # 最後に必ずモーターを止め、GPIOをリセットする
        motor_stop()
        if 'pwm_a' in globals():
            pwm_a.stop()
        GPIO.cleanup()
        print("GPIOを解放して終了しました")

if __name__ == "__main__":
    main()