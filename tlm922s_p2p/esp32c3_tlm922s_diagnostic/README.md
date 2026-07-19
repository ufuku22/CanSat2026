# ESP32-C3 TLM922S Diagnostic

TLM922SとのUART応答確認と、P2P設定の確認・修正だけを行うPlatformIOプロジェクトです。
受信処理は `../esp32c3_ground_station_receiver` 側に分離しています。

## 実行

1. VS Codeでこのフォルダを開く
2. PlatformIOの `Upload` を実行する
3. PlatformIOの `Monitor` を開く

起動時に `p2p get_*` で設定を確認し、期待値と違う場合は `p2p set_*` で修正して
`p2p save` します。
設定確認に成功するまでは10秒ごとに再試行します。設定確認に一度成功した後は、
設定の再確認は行わず、`mod get_ver` でUART応答だけを10秒ごとに確認します。

成功時の表示例:

```text
ESP32-C3 TLM922S diagnostic
Checking TLM922S UART and P2P settings...
Radio status: uart=ok p2p=configured saved=not_needed freq=922500000 pwr=14 sf=7 bw=125 cr=4/6 prlen=12 crc=on iqi=off sync=12
```
