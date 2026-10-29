#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MISSION_SCRIPT="${SCRIPT_DIR}/mission_arliss.py"
UNIT_NAME="${UNIT_NAME:-cansat-mission-arliss}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo so the mission can access the hardware." >&2
  echo "Example: sudo $0" >&2
  exit 1
fi

# root実行で保存先がroot所有にならないよう、毎回正常化する。
install -d -o argus -g argus -m 775 "${SCRIPT_DIR}/image_guidance_logs"

systemd-run \
  --unit="${UNIT_NAME}" \
  --collect \
  --working-directory="${SCRIPT_DIR}" \
  "${PYTHON_BIN}" "${MISSION_SCRIPT}"

echo "Started systemd unit: ${UNIT_NAME}"
echo "Follow logs with: journalctl -u ${UNIT_NAME} -f"
echo "Console logs: ${SCRIPT_DIR}/logs/mission_*_console.txt"
