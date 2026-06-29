# Wi-Fi Switcher for Raspberry Pi

USB-SSHでRaspberry Piに入っている状態で、Wi-Fi接続先を対話式に切り替えるためのスクリプトです。

既存のプロジェクトコードには依存しません。このフォルダ内の `switch_wifi.py` だけで動きます。追加のPythonパッケージや外部ライブラリは不要です。

## 想定する使い方

1. Raspberry PiとPCをUSB-SSHで接続する
2. Raspberry Pi上でこのフォルダへ移動する
3. スクリプトを実行する
4. AP一覧からスマホのテザリングSSIDを選ぶ
5. パスワードを入力する

```bash
cd ~/CanSat2026/wifi_switcher
sudo python3 switch_wifi.py
```

Wi-Fiインターフェース名が `wlan0` ではない場合:

```bash
sudo python3 switch_wifi.py --iface wlan0
```

## できること

- 周囲のAP探索
- 番号選択またはSSID直接入力
- パスワード非表示入力
- 接続切り替え
- IPアドレス取得確認
- `NetworkManager` 環境と `wpa_supplicant` 環境の両対応

## 注意

- Wi-Fi経由SSHで実行すると、切り替え時にSSHが切れます。USB-SSHから実行してください。
- `wpa_supplicant` 環境では `/etc/wpa_supplicant/wpa_supplicant.conf` を更新します。更新前に同じ場所へ日時付きバックアップを作ります。
- スマホ側のテザリングは、SSIDが見える状態にしてから実行してください。
