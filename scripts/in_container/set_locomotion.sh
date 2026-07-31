#!/usr/bin/env bash
set -euo pipefail

source /workspace/scripts/in_container/common.sh
cd "${RAICOM_DIR}/example/py"
python3 set_mode.py LD
