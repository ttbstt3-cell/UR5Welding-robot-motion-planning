#!/usr/bin/env bash
# Start CoppeliaSim with the ROS 2 Humble runtime visible to simROS2.
# Run this after saving/closing any existing CoppeliaSim instance.
# ROS 2 setup.bash intentionally reads a few variables before defining them;
# do not enable `set -u` before sourcing it.
set -eo pipefail

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# simROS2 is dlopen'ed by CoppeliaSim, so its dependencies must be visible in
# the process environment at startup (not only in a later terminal command).
export LD_LIBRARY_PATH="/opt/coppeliaSim:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

if [[ $# -eq 0 ]]; then
    set -- -f /home/ttbstt/jixiebi/UR5/ur5.ttt
fi

exec /opt/coppeliaSim/coppeliaSim.sh "$@"
