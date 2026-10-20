#!/usr/bin/env bash
# Start the stationary UR5 ROS 2 driver and the HMI with one command.
#
# Safety boundary: this script never powers on the robot, releases brakes, or
# sends a trajectory.  Once the operator has already powered the robot and
# started External Control, it may activate the configured trajectory
# controller.  Controller activation claims the command interface but does
# not command motion; all motion remains behind the HMI gates.
# ROS 2 Humble's setup.bash references AMENT_TRACE_SETUP_FILES before it is
# necessarily defined, so nounset must not be enabled here.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DRIVER_SCRIPT="$PROJECT_DIR/start_ur5_real.sh"
LOG_DIR="$PROJECT_DIR/logs"
DRIVER_LOG="$LOG_DIR/ur5_driver_startup.log"

source /opt/ros/humble/setup.bash
if [[ -f "$PROJECT_DIR/install/setup.bash" ]]; then
    source "$PROJECT_DIR/install/setup.bash"
fi

mkdir -p "$LOG_DIR"

DRIVER_STARTED=0
DRIVER_PID=""
DRIVER_PGID=""
DRIVER_RESTART_REQUIRED=0
HMI_PID=""
CONTROLLER_WATCH_PID=""
AUTO_ACTIVATE_CONTROLLER="${UR5_AUTO_ACTIVATE_CONTROLLER:-0}"

driver_launch_pattern='[/]opt/ros/humble/bin/ros2 launch ur_robot_driver ur_control.launch.py'

stop_existing_driver() {
    # Only use this path when the read-only health check proves that the
    # existing RTDE state is stale while Dashboard reports a live robot.
    # No robot command is sent; this only restarts the ROS 2 driver process.
    local launch_pid pgid
    launch_pid="$(pgrep -f "$driver_launch_pattern" | head -n 1 || true)"
    if [[ -z "$launch_pid" || "$launch_pid" == "$$" ]]; then
        return 1
    fi
    pgid="$(ps -o pgid= -p "$launch_pid" | tr -d ' ' || true)"
    echo "Stale RTDE state detected; restarting existing UR5 ROS 2 Driver (pid $launch_pid)."
    if [[ -n "$pgid" ]]; then
        kill -INT -- "-$pgid" 2>/dev/null || true
    else
        kill -INT "$launch_pid" 2>/dev/null || true
    fi
    for _ in {1..30}; do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
        if [[ -n "$pgid" ]]; then
            kill -TERM -- "-$pgid" 2>/dev/null || true
        else
            kill -TERM "$launch_pid" 2>/dev/null || true
        fi
    fi
    # Do not start a second listener until the old one has released both
    # reverse/script ports.
    for _ in {1..20}; do
        if ! ss -ltn 2>/dev/null | grep -qE ':50001|:50002'; then
            break
        fi
        sleep 0.5
    done
    return 0
}

driver_rtde_dashboard_mismatch() {
    # Compare read-only RTDE topic state with the Dashboard state.  A common
    # post-reboot failure is topic=POWER_OFF/program=false left by the old
    # RTDE process while Dashboard already reports RUNNING/true.
    local topic_mode dashboard_mode topic_program dashboard_program
    topic_mode="$(timeout 3 ros2 topic echo /io_and_status_controller/robot_mode --once 2>/dev/null \
        | sed -n 's/^mode: *//p' | head -n 1 || true)"
    topic_program="$(timeout 3 ros2 topic echo /io_and_status_controller/robot_program_running --once 2>/dev/null \
        | sed -n 's/^data: *//p' | head -n 1 || true)"
    dashboard_mode="$(timeout 3 ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode '{}' 2>/dev/null \
        | sed -n 's/.*robot_mode=.*mode=\([0-9][0-9]*\).*/\1/p' | head -n 1 || true)"
    dashboard_program="$(timeout 3 ros2 service call /dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning '{}' 2>/dev/null \
        | sed -n 's/.*program_running=\(True\|False\).*/\1/p' | head -n 1 || true)"

    if [[ "$topic_mode" =~ ^[0-9]+$ && "$dashboard_mode" =~ ^[0-9]+$ &&
          "$topic_program" =~ ^(true|false)$ && "$dashboard_program" =~ ^(True|False)$ ]]; then
        if [[ "$topic_mode" != "$dashboard_mode" ||
              "${topic_program,,}" != "${dashboard_program,,}" ]]; then
            echo "Driver health mismatch: RTDE mode=$topic_mode program=$topic_program; " \
                 "Dashboard mode=$dashboard_mode program=$dashboard_program." >&2
            return 0
        fi
    fi
    return 1
}

refresh_dashboard_connection() {
    # Dashboard reconnect is a transport/session repair only.  It does not
    # power on the robot, release brakes, activate a controller, or command
    # motion.  Ignore the call when the service is not available yet.
    timeout 4 ros2 service call /dashboard_client/connect std_srvs/srv/Trigger '{}' \
        >/dev/null 2>&1 || true
}

controller_is_active() {
    # ros2 control prints ANSI colours by default.  Strip them before parsing
    # so this remains reliable in both terminals and redirected logs.
    timeout 3 ros2 control list_controllers 2>/dev/null \
        | sed -E $'s/\\x1B\\[[0-9;]*[mK]//g' \
        | awk '$1 == "scaled_joint_trajectory_controller" && $3 == "active" { found=1 } END { exit(found ? 0 : 1) }'
}

controller_activation_ready() {
    local mode program safety scaling
    mode="$(timeout 3 ros2 topic echo /io_and_status_controller/robot_mode --once 2>/dev/null \
        | sed -n 's/^mode: *//p' | head -n 1 || true)"
    program="$(timeout 3 ros2 topic echo /io_and_status_controller/robot_program_running --once 2>/dev/null \
        | sed -n 's/^data: *//p' | head -n 1 || true)"
    safety="$(timeout 3 ros2 topic echo /io_and_status_controller/safety_mode --once 2>/dev/null \
        | sed -n 's/^mode: *//p' | head -n 1 || true)"
    scaling="$(timeout 3 ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once 2>/dev/null \
        | sed -n 's/^data: *//p' | head -n 1 || true)"

    [[ "$mode" == "7" ]] || return 1
    [[ "$program" == "true" ]] || return 1
    [[ "$safety" == "1" ]] || return 1
    awk -v value="$scaling" 'BEGIN { exit !(value ~ /^[0-9.]+$/ && value + 0 > 0) }'
}

controller_watchdog() {
    [[ "$AUTO_ACTIVATE_CONTROLLER" == "1" ]] || return 0
    echo "Controller watchdog enabled: waits for RUNNING + External Control + NORMAL safety." >&2
    while :; do
        if controller_is_active; then
            sleep 2
            continue
        fi

        if controller_activation_ready; then
            echo "Robot ready; activating scaled_joint_trajectory_controller (no motion command)." >&2
            if timeout 5 ros2 control switch_controllers \
                --activate scaled_joint_trajectory_controller \
                --strict >/dev/null 2>&1; then
                echo "scaled_joint_trajectory_controller is active." >&2
            else
                echo "Controller activation not available yet; will retry." >&2
            fi
        fi
        sleep 2
    done
}

cleanup() {
    if [[ -n "$CONTROLLER_WATCH_PID" ]] && kill -0 "$CONTROLLER_WATCH_PID" 2>/dev/null; then
        kill "$CONTROLLER_WATCH_PID" 2>/dev/null || true
    fi
    if [[ -n "$HMI_PID" ]] && kill -0 "$HMI_PID" 2>/dev/null; then
        kill -TERM "$HMI_PID" 2>/dev/null || true
    fi
    # Only stop a Driver instance created by this HMI invocation.  An already
    # running Driver belongs to the operator and is deliberately left alone.
    if [[ "$DRIVER_STARTED" -eq 1 && -n "$DRIVER_PID" ]]; then
        if kill -0 "$DRIVER_PID" 2>/dev/null; then
            if [[ -n "$DRIVER_PGID" ]]; then
                kill -INT -- "-$DRIVER_PGID" 2>/dev/null || true
            else
                kill -INT "$DRIVER_PID" 2>/dev/null || true
            fi
            for _ in {1..20}; do
                kill -0 "$DRIVER_PID" 2>/dev/null || break
                sleep 0.1
            done
            if kill -0 "$DRIVER_PID" 2>/dev/null; then
                if [[ -n "$DRIVER_PGID" ]]; then
                    kill -TERM -- "-$DRIVER_PGID" 2>/dev/null || true
                else
                    kill -TERM "$DRIVER_PID" 2>/dev/null || true
                fi
            fi
        fi
    fi
}
trap cleanup EXIT INT TERM

if pgrep -x ur_ros2_control_node >/dev/null 2>&1 || \
   ss -ltn 2>/dev/null | grep -q ':50002'; then
    refresh_dashboard_connection
    if driver_rtde_dashboard_mismatch; then
        if stop_existing_driver; then
            DRIVER_RESTART_REQUIRED=1
        fi
    else
        echo "Existing UR5 ROS 2 Driver detected; reusing it."
    fi
fi

need_start=0
if [[ "$DRIVER_RESTART_REQUIRED" -eq 1 ]]; then
    need_start=1
elif ! pgrep -x ur_ros2_control_node >/dev/null 2>&1 && \
     ! ss -ltn 2>/dev/null | grep -q ':50002'; then
    need_start=1
fi

if [[ "$need_start" -eq 1 ]]; then
    if [[ ! -x "$DRIVER_SCRIPT" ]]; then
        echo "WARNING: missing driver launcher: $DRIVER_SCRIPT" >&2
    else
        echo "Starting UR5 ROS 2 Driver; log: $DRIVER_LOG"
        # setsid gives the child Driver its own process group so cleanup does
        # not accidentally signal the HMI shell or another ROS process.
        setsid "$DRIVER_SCRIPT" >"$DRIVER_LOG" 2>&1 &
        DRIVER_PID=$!
        DRIVER_STARTED=1
        DRIVER_PGID="$(ps -o pgid= -p "$DRIVER_PID" | tr -d ' ')"
    fi
else
    echo "Existing UR5 ROS 2 Driver is ready; reusing it."
fi

# Give the driver time to negotiate RTDE and spawn controllers.  If the cable
# is absent, still open the HMI in monitor mode; the HMI will show the link as
# disconnected and keep all motion gates blocked.
ready=0
for _ in {1..60}; do
    if ss -ltn 2>/dev/null | grep -q ':50002'; then
        ready=1
        break
    fi
    sleep 1
done
if [[ "$ready" -eq 1 ]]; then
    echo "UR5 Driver listener ready."
else
    echo "WARNING: UR5 Driver is not ready; starting HMI in monitor-only state." >&2
fi

cd "$SCRIPT_DIR"
controller_watchdog &
CONTROLLER_WATCH_PID=$!

# Keep the HMI in the foreground for the operator, while the watchdog retries
# the controller switch after a reboot or External Control restart.
python3 main.py &
HMI_PID=$!
wait "$HMI_PID"
