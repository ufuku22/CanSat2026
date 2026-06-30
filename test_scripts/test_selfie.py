"""SelfieManagerの動作確認用スクリプト。"""

from pathlib import Path
import sys


# test_scripts の1つ上、つまりリポジトリ直下を import パスに追加する。
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from selfie_manager import SelfieManager  # noqa: E402


def log(message: str) -> None:
    """テストスクリプト用の簡易ログ出力。"""
    print(message, flush=True)


def main() -> None:
    """AP起動、撮影、画像保存、Wi-Fi復帰までを1回だけ実行する。"""
    log("SelfieManager test started")

    with SelfieManager() as selfie:
        saved_path = selfie.capture()

    log(f"test saved: {saved_path}")


if __name__ == "__main__":
    main()