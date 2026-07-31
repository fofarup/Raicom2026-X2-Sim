#!/usr/bin/env bash
set -euo pipefail

source /workspace/scripts/in_container/common.sh
python3 /workspace/control/safe_forward.py "$@"
