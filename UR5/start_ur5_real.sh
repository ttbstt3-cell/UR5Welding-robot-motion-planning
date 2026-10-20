#!/usr/bin/env bash
# Reliable, stationary-only startup for the UR5 ROS 2 driver.
#
# This script never powers on the robot, releases brakes, activates a motion
# controller, or sends a trajectory.  It only waits for the dedicated wired
# link and then starts the driver with explicit network/calibration settings.

set -eo pipefail

ROS_SETUP=/opt/ros/humble/setup.bash
PROFILE=${UR5_NM_PROFILE:-ur5-static}
IFACE=${UR5_INTERFACE:-eno1}
ROBOT_IP=${UR5_ROBOT_IP:-192.168.0.10}
PC_IP=${UR5_PC_IP:-192.168.0.20}
CALIBRATION=${UR5_CALIBRATION_FILE:-/home/ttbstt/jixiebi/UR5_real/ur5_calibration.yaml}
WAIT_SEC=${UR5_LINK_WAIT_SEC:-60}

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: missing ROS 2 setup: $ROS_SETUP" >&2
    exit 1
fi

# ROS Humble setup references variables before defining them.  Do not enable
# nounset before sourcing it.
source "$ROS_SETUP"

if [[ ! -f "$CALIBRATION" ]]; then
    echo "ERROR: calibration file not found: $CALIBRATION" >&2
    exit 1
fi

echo "Bringing up NetworkManager profile: $PROFILE"
nmcli connection up "$PROFILE" >/dev/null 2>&1 || true

echo "Waiting for $IFACE/$PC_IP and robot $ROBOT_IP (max ${WAIT_SEC}s)..."
ready=0
for ((i=0; i<WAIT_SEC; i++)); do
    has_ip=0
    if ip -4 addr show dev "$IFACE" | grep -q "inet ${PC_IP}/"; then
        has_ip=1
    fi
    if [[ "$has_ip" -eq 1 ]] && ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "$ready" -ne 1 ]]; then
    echo "ERROR: UR5 link is not ready." >&2
    echo "Check the Ethernet cable/switch and NetworkManager profile '$PROFILE'." >&2
    ip link show "$IFACE" >&2 || true
    ip -4 addr show dev "$IFACE" >&2 || true
    exit 2
fi

echo "Network ready: $(ip -4 -o addr show dev "$IFACE" | tr '\n' ' ')"
echo "Starting stationary ROS 2 driver for $ROBOT_IP"

exec ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 \
    robot_ip:="$ROBOT_IP" \
    reverse_ip:="$PC_IP" \
    reverse_port:=50001 \
    script_sender_port:=50002 \
    kinematics_params_file:="$CALIBRATION" \
    launch_rviz:=false \
    launch_dashboard_client:=true \
    verify_robot_model:=true \
    activate_joint_controller:=false \
    headless_mode:=false
