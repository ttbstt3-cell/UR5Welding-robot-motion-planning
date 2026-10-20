#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="/opt/coppeliaSim:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
exec /opt/coppeliaSim/coppeliaSim -GpreferredSandboxLang=lua -GzmqRemoteApi.rpcPort=23010 "-csim=require('sim');sim.loadScene('$TASK_ROOT/simulation/cr7.ttt')" "$@"
