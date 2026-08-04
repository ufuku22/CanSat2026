#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TEST_SCRIPT="${REPO_DIR}/test_scripts/noshiro_selfie_mission_test.py"
UNIT_NAME="${UNIT_NAME:-cansat-noshiro-selfie-mission-test}"
LOG_FILE="${REPO_DIR}/logs/noshiro_selfie_mission_test.log"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Wi-Fiを切り替えるためsudoで実行してください。" >&2
  echo "実行例: sudo $0" >&2
  exit 1
fi

systemd-run \
  --unit="${UNIT_NAME}" \
  --collect \
  --working-directory="${REPO_DIR}" \
  "${PYTHON_BIN}" "${TEST_SCRIPT}" "$@"

echo "Started systemd unit: ${UNIT_NAME}"
echo "Follow logs with: journalctl -u ${UNIT_NAME} -f"
echo "Event log: ${LOG_FILE}"
