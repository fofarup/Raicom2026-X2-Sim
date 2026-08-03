#!/usr/bin/env bash
set -euo pipefail

sim_root="${1:-/workspace/.runtime/raicom2026/sim_mujoco}"
output="${2:-/workspace/.runtime/raicom2026/project_plugins/libsensor_ray.so}"
include_root="$(python3 -c 'import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).parent / "include")')"
mkdir -p "$(dirname "${output}")"
g++ -std=c++17 -O2 -fPIC -shared \
  -I"${include_root}" \
  /workspace/sim/raycaster_lidar_fixed.cc \
  -L"${sim_root}/lib" -Wl,-rpath,"${sim_root}/lib" -lmujoco \
  -o "${output}"
echo "${output}"
