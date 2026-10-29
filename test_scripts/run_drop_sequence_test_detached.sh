#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TEST_SCRIPT="${REPO_DIR}/test_scripts/drop_sequence_test.py"
UNIT_NAME="${UNIT_NAME:-cansat-drop-sequence-test}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo so the test can access the hardware." >&2
  echo "Example: sudo $0 25" >&2
  exit 1
fi

AIR_TEMPERATURE_C="${1:-}"
if [[ -z "${AIR_TEMPERATURE_C}" ]]; then
  read -r -p "Air temperature [°C]: " AIR_TEMPERATURE_C
fi

if ! "${PYTHON_BIN}" -c \
  'import sys; value = float(sys.argv[1]); sys.exit(value <= -273.15)' \
  "${AIR_TEMPERATURE_C}"; then
  echo "Air temperature must be a number greater than -273.15°C." >&2
  exit 1
fi

systemd-run \
  --unit="${UNIT_NAME}" \
  --collect \
  --working-directory="${REPO_DIR}" \
  /bin/sh -c 'printf "%s\n" "$1" | exec "$2" "$3"' \
  sh "${AIR_TEMPERATURE_C}" "${PYTHON_BIN}" "${TEST_SCRIPT}"

echo "Started systemd unit: ${UNIT_NAME}"
echo "Follow logs with: journalctl -u ${UNIT_NAME} -f"
echo "Event logs: ${REPO_DIR}/logs/drop_*.txt"
echo "Sensor CSV logs: ${REPO_DIR}/sensor_csv_logs/drop_*.csv"
