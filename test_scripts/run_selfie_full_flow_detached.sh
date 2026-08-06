#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TEST_SCRIPT="${REPO_DIR}/test_scripts/test_selfie_full_flow.py"
UNIT_NAME="${UNIT_NAME:-cansat-selfie-full-flow-test}"
LOG_FILE="${REPO_DIR}/logs/selfie_full_flow.log"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo so the test can switch Wi-Fi AP settings." >&2
  echo "Example: sudo $0 --ev 0.0" >&2
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
echo "Exposure option: --ev {-1.0,-0.5,0.0,0.5,1.0} (omitted: auto)"
