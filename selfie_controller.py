#!/usr/bin/env python3
"""CanSat2026 の自撮りアームと ESP32-S3 Sense カメラを制御するファイル。

FIT0579 モータは TI DRV8838 モータドライバで動かします。DRV8838 は PH/EN
方式で、PH が回転方向、EN が H ブリッジの有効化を担当します。速度や保持トルク
を下げたいときは EN に PWM をかけます。

カメラ側は Wi-Fi 経由で JPEG 撮影エンドポイントを公開している想定です。
たとえば ESP32 camera web server の ``/capture`` を使えます。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Literal, Protocol
from urllib.parse import urljoin
from urllib.request import Request, urlopen


MotorDirection = Literal["forward", "reverse"]
ArmState = Literal["stowed", "deploying", "deployed", "holding", "retracting", "stopped"]
HoldMode = Literal["drive", "brake"]
DRV8838_WAKE_SECONDS = 0.001


@dataclass(frozen=True)
class Drv8838Pins:
    """DRV8838 に接続する Raspberry Pi の GPIO ピン番号をまとめるクラス。

    デフォルトでは物理ピン番号ではなく BCM GPIO 番号を使います。基板側で
    nSLEEP を High に固定している場合、``sleep`` は省略できます。
    """

    phase: int
    enable: int
    sleep: int | None = None


@dataclass(frozen=True)
class SelfieControllerConfig:
    """自撮りアームの動作時間、PWM 出力、カメラ URL などの設定をまとめるクラス。"""

    motor_pins: Drv8838Pins
    esp32_base_url: str
    capture_path: str = "/capture"
    capture_dir: str | Path = "captures"
    gpio_mode: str = "BCM"
    pwm_frequency_hz: int = 1000
    forward_phase_high: bool = False
    deploy_seconds: float = 2.0
    retract_seconds: float = 2.0
    deploy_duty_cycle: float = 80.0
    retract_duty_cycle: float = 80.0
    hold_duty_cycle: float = 25.0
    hold_mode: HoldMode = "drive"
    request_timeout: float = 10.0


class MotorDriver(Protocol):
    """実機用・テスト用のモータドライバを同じ形で扱うためのインターフェース。"""

    def setup(self) -> None:
        """モータ出力に使う GPIO や PWM を初期化する。"""

    def run(self, direction: MotorDirection, duty_cycle: float) -> None:
        """指定した方向と PWM デューティ比でモータを回す。"""

    def brake(self) -> None:
        """モータをブレーキ状態にして、軸が動きにくい状態にする。"""

    def coast(self) -> None:
        """モータへの駆動を止め、空転できる停止状態にする。"""

    def close(self) -> None:
        """GPIO や PWM など、使っていたハードウェア資源を解放する。"""


class Drv8838MotorDriver:
    """Raspberry Pi の GPIO で DRV8838 の PH/EN 制御を行うクラス。"""

    def __init__(
        self,
        pins: Drv8838Pins,
        pwm_frequency_hz: int = 1000,
        gpio_mode: str = "BCM",
        forward_phase_high: bool = False,
        gpio_module: Any | None = None,
    ) -> None:
        """DRV8838 のピン設定と PWM 条件を保存し、初期状態を未接続にする。

        ``gpio_module`` を渡すと、実機 GPIO の代わりにテスト用のモジュールを
        差し込めます。
        """
        self.pins = pins
        self.pwm_frequency_hz = pwm_frequency_hz
        self.gpio_mode = gpio_mode
        self.forward_phase_high = forward_phase_high
        self.gpio = gpio_module
        self.pwm: Any | None = None
        self._is_setup = False

    def setup(self) -> None:
        """GPIO モード、PH/EN/nSLEEP ピン、EN 用 PWM を初期化する。

        初期化済みなら何もしません。nSLEEP ピンが設定されている場合は High にして
        DRV8838 を起こし、最後にブレーキ状態にします。
        """
        if self._is_setup:
            return

        gpio = self.gpio or _load_gpio()
        self.gpio = gpio
        mode = getattr(gpio, self.gpio_mode)
        gpio.setmode(mode)
        gpio.setup(self.pins.phase, gpio.OUT)
        gpio.setup(self.pins.enable, gpio.OUT)
        if self.pins.sleep is not None:
            gpio.setup(self.pins.sleep, gpio.OUT)
            gpio.output(self.pins.sleep, gpio.HIGH)
            time.sleep(DRV8838_WAKE_SECONDS)

        self.pwm = gpio.PWM(self.pins.enable, self.pwm_frequency_hz)
        self.pwm.start(0)
        self._is_setup = True
        self.brake()

    def run(self, direction: MotorDirection, duty_cycle: float) -> None:
        """DRV8838 を起こし、PH で方向を決めて EN の PWM でモータを回す。

        ``direction`` が ``forward`` なら展開方向、``reverse`` なら収納方向として
        扱います。実際の回転方向が逆の場合は設定の ``forward_phase_high`` を
        反転してください。
        """
        self.setup()
        self._wake()
        duty = _clamp_duty_cycle(duty_cycle)
        gpio = self._require_gpio()

        if direction == "forward":
            phase_high = self.forward_phase_high
        elif direction == "reverse":
            phase_high = not self.forward_phase_high
        else:
            raise ValueError(f"unsupported motor direction: {direction!r}")

        gpio.output(self.pins.phase, gpio.HIGH if phase_high else gpio.LOW)
        self._require_pwm().ChangeDutyCycle(duty)

    def brake(self) -> None:
        """DRV8838 をブレーキ状態にする。

        DRV8838 では EN を Low にするとブレーキ状態になります。アームを止めたい
        ときや、短時間だけ位置を保ちたいときに使います。
        """
        self.setup()
        self._wake()
        gpio = self._require_gpio()
        self._require_pwm().ChangeDutyCycle(0)
        gpio.output(self.pins.phase, gpio.LOW)

    def coast(self) -> None:
        """モータを空転停止にする。

        nSLEEP ピンを制御している場合は Low にして DRV8838 をスリープさせます。
        nSLEEP を配線していない場合は EN の PWM を 0 にするだけです。
        """
        self.setup()
        gpio = self._require_gpio()
        self._require_pwm().ChangeDutyCycle(0)
        if self.pins.sleep is not None:
            gpio.output(self.pins.sleep, gpio.LOW)

    def close(self) -> None:
        """モータ出力を止め、PWM を停止し、使用した GPIO ピンを解放する。"""
        if self.gpio and self._is_setup and self.pins.sleep is not None:
            self.gpio.output(self.pins.sleep, self.gpio.LOW)
        if self.pwm:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
            self.pwm = None
        if self.gpio and self._is_setup:
            pins = [self.pins.phase, self.pins.enable]
            if self.pins.sleep is not None:
                pins.append(self.pins.sleep)
            self.gpio.cleanup(pins)
        self._is_setup = False

    def _wake(self) -> None:
        """nSLEEP ピンがある場合に High へ戻し、DRV8838 を動作可能状態にする。"""
        if self.pins.sleep is None:
            return
        gpio = self._require_gpio()
        gpio.output(self.pins.sleep, gpio.HIGH)
        time.sleep(DRV8838_WAKE_SECONDS)

    def _require_gpio(self) -> Any:
        """GPIO モジュールが初期化済みであることを確認して返す。"""
        if self.gpio is None:
            raise RuntimeError("GPIO module is not loaded")
        return self.gpio

    def _require_pwm(self) -> Any:
        """PWM オブジェクトが初期化済みであることを確認して返す。"""
        if self.pwm is None:
            raise RuntimeError("PWM is not initialized")
        return self.pwm


class SelfieController:
    """自撮りアームのモータと ESP32-S3 Sense Wi-Fi カメラをまとめて扱うクラス。"""

    def __init__(
        self,
        config: SelfieControllerConfig,
        motor_driver: MotorDriver | None = None,
    ) -> None:
        """設定を保存し、モータドライバとアーム状態の初期値を用意する。

        ``motor_driver`` を省略した場合は DRV8838 用の実機ドライバを使います。
        テストでは fake driver を渡すことで GPIO なしでも動作確認できます。
        """
        self.config = config
        self.motor_driver = motor_driver or Drv8838MotorDriver(
            pins=config.motor_pins,
            pwm_frequency_hz=config.pwm_frequency_hz,
            gpio_mode=config.gpio_mode,
            forward_phase_high=config.forward_phase_high,
        )
        self.arm_state: ArmState = "stowed"
        self.last_photo_path: Path | None = None
        self.last_action_at: str | None = None

    def __enter__(self) -> "SelfieController":
        """with 文に入るときにモータ GPIO を初期化し、自分自身を返す。"""
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        """with 文を抜けるときにモータを止め、GPIO 資源を解放する。"""
        self.close()

    def open(self) -> None:
        """モータを使う前に GPIO と PWM を初期化する。"""
        self.motor_driver.setup()

    def close(self) -> None:
        """モータを停止し、GPIO と PWM を解放する。"""
        self.motor_driver.close()

    def deploy_arm(
        self,
        seconds: float | None = None,
        duty_cycle: float | None = None,
        stop_after: bool = True,
    ) -> dict[str, Any]:
        """FIT0579 モータを展開方向に回して、自撮りアームを外へ出す。

        秒数と PWM デューティ比を省略すると設定値を使います。``stop_after`` が True
        の場合、指定時間だけ回したあとブレーキをかけます。
        """
        run_seconds = self.config.deploy_seconds if seconds is None else seconds
        duty = self.config.deploy_duty_cycle if duty_cycle is None else duty_cycle
        self._run_timed_motor(
            direction="forward",
            seconds=run_seconds,
            duty_cycle=duty,
            state_while_running="deploying",
            stop_after=stop_after,
        )
        self.arm_state = "deployed" if stop_after else "deploying"
        return self.read_all()

    def hold_arm(self, duty_cycle: float | None = None) -> dict[str, Any]:
        """展開済みのアームを維持する。

        ``hold_mode="drive"`` なら弱い展開方向トルクをかけ続けます。
        ``hold_mode="brake"`` なら DRV8838 のブレーキ状態で保持します。
        """
        if self.config.hold_mode == "brake":
            self.motor_driver.brake()
        else:
            duty = self.config.hold_duty_cycle if duty_cycle is None else duty_cycle
            self.motor_driver.run("forward", duty)

        self.arm_state = "holding"
        self._touch()
        return self.read_all()

    def retract_arm(
        self,
        seconds: float | None = None,
        duty_cycle: float | None = None,
        stop_after: bool = True,
    ) -> dict[str, Any]:
        """FIT0579 モータを収納方向に回して、自撮りアームを戻す。

        秒数と PWM デューティ比を省略すると設定値を使います。``stop_after`` が True
        の場合、指定時間だけ回したあとブレーキをかけます。
        """
        run_seconds = self.config.retract_seconds if seconds is None else seconds
        duty = self.config.retract_duty_cycle if duty_cycle is None else duty_cycle
        self._run_timed_motor(
            direction="reverse",
            seconds=run_seconds,
            duty_cycle=duty,
            state_while_running="retracting",
            stop_after=stop_after,
        )
        self.arm_state = "stowed" if stop_after else "retracting"
        return self.read_all()

    def stop_arm(self, brake: bool = True) -> dict[str, Any]:
        """アーム用モータを停止する。

        ``brake`` が True ならブレーキ停止、False なら空転停止にします。
        """
        if brake:
            self.motor_driver.brake()
        else:
            self.motor_driver.coast()
        self.arm_state = "stopped"
        self._touch()
        return self.read_all()

    def capture_photo(
        self,
        filename: str | None = None,
        capture_url: str | None = None,
    ) -> Path:
        """ESP32-S3 Sense カメラへ HTTP リクエストを送り、JPEG 写真を 1 枚保存する。

        ``filename`` を省略すると日時入りのファイル名を自動生成します。
        ``capture_url`` を渡すと、設定の base URL と capture path の代わりに
        その URL へ直接アクセスします。
        """
        url = capture_url or self._capture_url()
        request = Request(url, headers={"User-Agent": "CanSat2026-SelfieController"})

        with urlopen(request, timeout=self.config.request_timeout) as response:
            image_bytes = response.read()
            content_type = response.headers.get("Content-Type", "")

        if not image_bytes:
            raise RuntimeError("ESP32 camera returned an empty response")
        if content_type and "image" not in content_type.lower():
            raise RuntimeError(f"ESP32 camera did not return an image: {content_type}")

        capture_dir = Path(self.config.capture_dir)
        capture_dir.mkdir(parents=True, exist_ok=True)
        photo_name = filename or f"selfie_{datetime.now():%Y%m%d_%H%M%S}.jpg"
        destination = capture_dir / Path(photo_name).name
        destination.write_bytes(image_bytes)

        self.last_photo_path = destination
        self._touch()
        return destination

    def read_all(self) -> dict[str, Any]:
        """現在のアーム状態や最後に撮影したファイルをログ用の dict として返す。

        ``Logger.register_source()`` にそのまま渡せる形にしています。
        """
        return {
            "arm_state": self.arm_state,
            "last_photo_path": str(self.last_photo_path) if self.last_photo_path else None,
            "last_action_at": self.last_action_at,
            "esp32_base_url": self.config.esp32_base_url,
        }

    def _run_timed_motor(
        self,
        direction: MotorDirection,
        seconds: float,
        duty_cycle: float,
        state_while_running: ArmState,
        stop_after: bool,
    ) -> None:
        """指定方向へ一定時間モータを回し、必要なら最後にブレーキをかける。

        ``deploy_arm()`` と ``retract_arm()`` から共通利用する内部関数です。
        """
        if seconds < 0:
            raise ValueError("seconds must be greater than or equal to 0")

        self.arm_state = state_while_running
        self._touch()
        self.motor_driver.run(direction, duty_cycle)
        if seconds:
            time.sleep(seconds)
        if stop_after:
            self.motor_driver.brake()

    def _capture_url(self) -> str:
        """ESP32 カメラの base URL と撮影パスから、実際にアクセスする URL を作る。"""
        if not self.config.esp32_base_url:
            raise ValueError("esp32_base_url must be set")
        return urljoin(self.config.esp32_base_url.rstrip("/") + "/", self.config.capture_path.lstrip("/"))

    def _touch(self) -> None:
        """最後に操作した時刻を ISO 形式の文字列で記録する。"""
        self.last_action_at = datetime.now().isoformat(timespec="seconds")


def _load_gpio() -> Any:
    """Raspberry Pi 用の RPi.GPIO を読み込み、使えない場合は分かりやすい例外を出す。"""
    try:
        import RPi.GPIO as gpio
    except ImportError as exc:
        raise RuntimeError(
            "RPi.GPIO is required to drive the arm motor on Raspberry Pi. "
            "Install it on the Raspberry Pi or inject a test motor_driver."
        ) from exc
    return gpio


def _clamp_duty_cycle(value: float) -> float:
    """PWM デューティ比を 0.0 から 100.0 の範囲に丸める。"""
    return max(0.0, min(100.0, float(value)))
