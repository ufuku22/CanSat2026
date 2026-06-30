# ESP32-C3 USB Bridge

ESP32-C3をUSB-UARTブリッジとして使い、PCからTLM922Sコマンドを送るための
PlatformIOプロジェクトです。

## 実行

1. VS Codeでこのフォルダを開く
2. PlatformIOの `Upload` を実行する
3. PlatformIO Monitorではなく、PC側で `usb_bridge_monitor.py` を起動する

通常はCOMポートを省略できます。

```bash
python usb_bridge_monitor.py
```

自動判定がうまくいかない場合は明示します。

```bash
python usb_bridge_monitor.py --port COM4
```

起動後、同じ画面にTLM922Sコマンドを入力してEnterを押します。

```text
mod get_ver
p2p get_freq
p2p set_freq 922500000
p2p save
```

シリアルログは `usb_bridge_logs/raw_serial_*.log` に保存されます。
