#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 "$TASK_ROOT/scripts/cr7_controller.py" "$@"
