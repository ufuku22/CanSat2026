#!/usr/bin/env bash
set -euo pipefail

MODE="nohup"
if [[ "${1:-}" == "--systemd" ]]; then
  MODE="systemd"
  shift
elif [[ "${1:-}" == "--nohup" ]]; then
  shift
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${REPO_DIR}/logs"
LOG_FILE="${LOG_DIR}/esp32s3_camera_receiver.log"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TEST_SCRIPT="${REPO_DIR}/test_scripts/test_esp32s3_camera_receiver.py"

mkdir -p "${LOG_DIR}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo so the test can switch Wi-Fi AP settings." >&2
  echo "Example: sudo $0 [--nohup|--systemd]" >&2
  exit 1
fi

if [[ "${MODE}" == "systemd" ]]; then
  UNIT_NAME="${UNIT_NAME:-cansat-camera-test}"
  systemd-run \
    --unit="${UNIT_NAME}" \
    --collect \
    --working-directory="${REPO_DIR}" \
    "${PYTHON_BIN}" "${TEST_SCRIPT}" "$@"
  echo "Started systemd unit: ${UNIT_NAME}"
  echo "Follow logs with: journalctl -u ${UNIT_NAME} -f"
else
  nohup "${PYTHON_BIN}" "${TEST_SCRIPT}" "$@" >"${LOG_FILE}" 2>&1 </dev/null &
  PID="$!"
  echo "${PID}" >"${LOG_DIR}/esp32s3_camera_receiver.pid"
  echo "Started detached test with PID ${PID}"
  echo "Log: ${LOG_FILE}"
  echo "Follow logs with: tail -f ${LOG_FILE}"
fi
