#!/usr/bin/env bash
set -euo pipefail
helper_native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
helper_bin_dir="${helper_native_dir}/../bin"
mkdir -p -- "${helper_bin_dir}"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I/opt/ros/humble/include \
  "${helper_native_dir}/robot_tcp_helper.cpp" \
  -L/opt/ros/humble/lib/x86_64-linux-gnu -lurcl \
  -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu -pthread \
  -o "${helper_bin_dir}/robot_tcp_helper"
"${helper_bin_dir}/robot_tcp_helper" --self-test
