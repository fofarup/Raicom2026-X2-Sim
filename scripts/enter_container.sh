#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/common.sh"

require_container
exec docker exec -it \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
  "${DOCKER_CONTAINER}" bash
