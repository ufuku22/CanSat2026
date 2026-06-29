#!/usr/bin/env bash
set -u

# Raspberry Pi Wi-Fi switcher for USB-SSH operation.
# 外部パッケージを追加せず、OS標準の NetworkManager / wpa_supplicant 系ツールを使う。

SCRIPT_NAME="$(basename "$0")"
TEMP_DIR="${TMPDIR:-/tmp}"
SCAN_FILE="$TEMP_DIR/wifi_switcher_scan.$$"

cleanup() {
  rm -f "$SCAN_FILE"
}
trap cleanup EXIT

die() {
  echo "ERROR: $*" >&2
  exit 1
}

has_cmd() {
  command -v "$1" >/dev/null 2>&1
}

need_root() {
  if [ "$(id -u)" -ne 0 ]; then
    if has_cmd sudo; then
      exec sudo -- "$0" "$@"
    fi
    die "root権限が必要です。sudo ./$SCRIPT_NAME で実行してください。"
  fi
}

detect_iface() {
  if [ -n "${WIFI_IFACE:-}" ]; then
    echo "$WIFI_IFACE"
    return
  fi

  if [ -d /sys/class/net/wlan0 ]; then
    echo "wlan0"
    return
  fi

  for path in /sys/class/net/wl*; do
    [ -e "$path" ] || continue
    basename "$path"
    return
  done

  die "Wi-Fiインターフェースが見つかりません。例: WIFI_IFACE=wlan0 ./$SCRIPT_NAME"
}

detect_backend() {
  if has_cmd nmcli && nmcli -t -f RUNNING general 2>/dev/null | grep -q '^running$'; then
    echo "nmcli"
    return
  fi

  if has_cmd wpa_cli && has_cmd wpa_passphrase; then
    echo "wpa"
    return
  fi

  die "nmcli または wpa_cli/wpa_passphrase が見つかりません。"
}

read_secret() {
  prompt="$1"
  value=""
  if [ -t 0 ]; then
    printf "%s" "$prompt"
    stty -echo
    IFS= read -r value
    stty echo
    printf "\n"
  else
    IFS= read -r value
  fi
  echo "$value"
}

print_header() {
  echo
  echo "=== Raspberry Pi Wi-Fi Switcher ==="
  echo "interface: $IFACE"
  echo "backend  : $BACKEND"
  echo "USB-SSH接続中に使う前提です。Wi-Fi側のSSH接続から実行すると切断されます。"
  echo
}

scan_nmcli() {
  nmcli radio wifi on >/dev/null 2>&1 || true
  nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1 || true
  sleep 2
  nmcli -t --escape no -f SSID,SIGNAL,SECURITY device wifi list ifname "$IFACE" |
    awk -F: 'length($1) > 0 && !seen[$1]++ { printf "%s\t%s\t%s\n", $1, $2, $3 }' > "$SCAN_FILE"
}

scan_wpa() {
  ip link set "$IFACE" up >/dev/null 2>&1 || true
  wpa_cli -i "$IFACE" scan >/dev/null 2>&1 || true
  sleep 3
  wpa_cli -i "$IFACE" scan_results 2>/dev/null |
    awk 'NR > 2 {
      ssid = $5
      for (i = 6; i <= NF; i++) ssid = ssid " " $i
      if (length(ssid) > 0 && !seen[ssid]++) printf "%s\t%s\t%s\n", ssid, $3, $4
    }' > "$SCAN_FILE"
}

scan_networks() {
  echo "APを探索しています..."
  if [ "$BACKEND" = "nmcli" ]; then
    scan_nmcli
  else
    scan_wpa
  fi

  if [ ! -s "$SCAN_FILE" ]; then
    echo "APが見つかりませんでした。SSIDを手入力できます。"
    return 1
  fi

  echo
  echo "見つかったAP:"
  awk -F'\t' '{ printf "  %2d) %-32s signal:%-4s security:%s\n", NR, $1, $2, $3 }' "$SCAN_FILE"
  return 0
}

select_ssid() {
  echo
  printf "接続する番号、またはSSIDを直接入力してください: "
  IFS= read -r choice

  if [ -z "$choice" ]; then
    die "SSIDが空です。"
  fi

  case "$choice" in
    *[!0-9]*)
      echo "$choice"
      ;;
    *)
      if [ ! -s "$SCAN_FILE" ]; then
        echo "$choice"
        return
      fi
      ssid="$(awk -F'\t' -v n="$choice" 'NR == n { print $1 }' "$SCAN_FILE")"
      [ -n "$ssid" ] || die "指定された番号のAPがありません。"
      echo "$ssid"
      ;;
  esac
}

connect_nmcli() {
  ssid="$1"
  password="$2"

  if [ -z "$password" ]; then
    nmcli device wifi connect "$ssid" ifname "$IFACE"
  else
    nmcli device wifi connect "$ssid" password "$password" ifname "$IFACE"
  fi
}

connect_wpa() {
  ssid="$1"
  password="$2"
  conf="/etc/wpa_supplicant/wpa_supplicant.conf"

  [ -f "$conf" ] || die "$conf が見つかりません。"
  cp "$conf" "$conf.bak.$(date +%Y%m%d_%H%M%S)" || die "設定ファイルのバックアップに失敗しました。"

  tmp_conf="$TEMP_DIR/wpa_supplicant_new.$$"
  awk -v target="$ssid" '
    BEGIN { skip = 0; depth = 0 }
    /^[[:space:]]*network=\{/ { block = $0 "\n"; skip = 0; depth = 1; next }
    depth > 0 {
      block = block $0 "\n"
      if (index($0, "ssid=\"" target "\"") > 0) skip = 1
      if ($0 ~ /^[[:space:]]*\}/) {
        if (!skip) printf "%s", block
        block = ""; depth = 0; skip = 0
      }
      next
    }
    { print }
  ' "$conf" > "$tmp_conf" || die "既存設定の読み取りに失敗しました。"

  if [ -z "$password" ]; then
    {
      echo
      echo "network={"
      echo "    ssid=\"$ssid\""
      echo "    key_mgmt=NONE"
      echo "}"
    } >> "$tmp_conf"
  else
    wpa_passphrase "$ssid" "$password" >> "$tmp_conf" || die "Wi-Fi設定の生成に失敗しました。"
  fi

  install -m 600 "$tmp_conf" "$conf" || die "Wi-Fi設定の保存に失敗しました。"
  rm -f "$tmp_conf"

  wpa_cli -i "$IFACE" reconfigure >/dev/null 2>&1 || true
  if has_cmd dhclient; then
    dhclient -r "$IFACE" >/dev/null 2>&1 || true
    dhclient "$IFACE" >/dev/null 2>&1 || true
  elif has_cmd systemctl; then
    systemctl restart dhcpcd >/dev/null 2>&1 || true
  fi
}

show_status() {
  echo
  echo "接続状態:"
  if [ "$BACKEND" = "nmcli" ]; then
    nmcli -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show "$IFACE" 2>/dev/null || true
  else
    wpa_cli -i "$IFACE" status 2>/dev/null | grep -E '^(wpa_state|ssid|ip_address)=' || true
    ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet / { print "ip_address=" $2 }' || true
  fi
}

wait_for_connection() {
  echo
  echo "接続完了を待っています..."
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    if ip -4 addr show "$IFACE" 2>/dev/null | grep -q ' inet '; then
      echo "IPアドレスを取得しました。"
      show_status
      return 0
    fi
    sleep 2
  done

  echo "まだIPアドレスを確認できません。パスワードや電波状況を確認してください。"
  show_status
  return 1
}

usage() {
  cat <<EOF
使い方:
  sudo ./$SCRIPT_NAME
  sudo WIFI_IFACE=wlan0 ./$SCRIPT_NAME

内容:
  AP探索 → 番号選択またはSSID入力 → パスワード入力 → 接続確認
EOF
}

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

need_root "$@"
IFACE="$(detect_iface)"
BACKEND="$(detect_backend)"

print_header
scan_networks || true
SSID="$(select_ssid)"
PASSWORD="$(read_secret "パスワードを入力してください（オープンAPなら空Enter）: ")"

echo
echo "接続を切り替えます: $SSID"
if [ "$BACKEND" = "nmcli" ]; then
  connect_nmcli "$SSID" "$PASSWORD" || die "接続に失敗しました。"
else
  connect_wpa "$SSID" "$PASSWORD"
fi

wait_for_connection
