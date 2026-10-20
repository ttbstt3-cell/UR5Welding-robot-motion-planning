from __future__ import annotations

import math
import os
import socket
import time
from typing import Callable, Dict, Optional

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import IsProgramRunning

from ros.state_monitor import RobotState
from ros.trajectory_client import TrajectoryClient


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float):
    # ROS quaternion order is x,y,z,w. Return roll,pitch,yaw in radians.
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return [roll, pitch, yaw]


class UR5RosNode(Node):
    def __init__(
        self,
        on_state: Callable[[RobotState], None],
        on_log: Callable[[str], None],
        on_goal_active: Callable[[bool], None],
        on_goal_done: Callable[[bool, str], None],
        action_name: str,
        on_motion: Optional[Callable[[str, bool, str], None]] = None,
        robot_ip: Optional[str] = None,
    ):
        super().__init__("ur5_safe_hmi")
        self.on_state = on_state
        self.on_log = on_log
        self.state = RobotState()
        self.robot_ip = str(robot_ip or os.environ.get("UR5_ROBOT_IP", "192.168.0.10"))
        self._last_status_emit = 0.0
        self._controller_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        # Some CB3/External-Control combinations publish
        # ``robot_program_running`` only on a state transition.  After a
        # driver restart there may therefore be no latched sample yet even
        # though the dashboard already knows the current value.  Keep the
        # topic as the primary source, and use this read-only dashboard
        # service as a startup/reconnect fallback so the HMI safety gate does
        # not remain stale after a restart.
        self._program_client = self.create_client(
            IsProgramRunning, "/dashboard_client/program_running"
        )
        self._program_future = None
        # Motion lifecycle logs share the HMI log channel so goal ids are
        # visible in the same view as the rest of the operator messages.
        self.trajectory = TrajectoryClient(
            self,
            action_name,
            on_goal_active,
            on_goal_done,
            on_motion=on_motion,
            on_log=on_log,
        )
        self._goal_active_cb = on_goal_active

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, "/io_and_status_controller/robot_program_running", self._program_cb, status_qos)
        self.create_subscription(RobotMode, "/io_and_status_controller/robot_mode", self._mode_cb, status_qos)
        self.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode", self._safety_cb, status_qos)
        self.create_subscription(Float64, "/speed_scaling_state_broadcaster/speed_scaling", self._speed_cb, status_qos)
        self.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", self._tcp_cb, 10)
        self.create_subscription(String, "/robot_description", self._description_cb, status_qos)
        self.create_timer(1.0, self._poll_status)
        self._emit("ROS 2 HMI node started")

    def _emit(self, message: str) -> None:
        self.on_log(message)

    def _publish_state(self) -> None:
        self.state.action_server_ready = self.trajectory.ready()
        self.on_state(self.state.copy())

    def _joint_cb(self, msg: JointState) -> None:
        values: Dict[str, float] = dict(zip(msg.name, msg.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        self.state.joint_names = list(JOINT_NAMES)
        self.state.positions = [float(values[name]) for name in JOINT_NAMES]
        self.state.joint_stamp = time.monotonic()
        self._publish_state()

    def _program_cb(self, msg: Bool) -> None:
        self.state.program_running = bool(msg.data)
        self._publish_state()

    def _mode_cb(self, msg: RobotMode) -> None:
        self.state.robot_mode = int(msg.mode)
        self._publish_state()

    def _safety_cb(self, msg: SafetyMode) -> None:
        self.state.safety_mode = int(msg.mode)
        self._publish_state()

    def _speed_cb(self, msg: Float64) -> None:
        self.state.speed_scaling = float(msg.data)
        self._publish_state()

    def _tcp_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        quat = [float(q.x), float(q.y), float(q.z), float(q.w)]
        self.state.tcp_position = [float(p.x), float(p.y), float(p.z)]
        self.state.tcp_frame_id = str(msg.header.frame_id or "")
        self.state.tcp_stamp = time.monotonic()
        self.state.tcp_quaternion = quat
        self.state.tcp_rpy = _quat_to_rpy(*quat)
        self._publish_state()

    def _description_cb(self, msg: String) -> None:
        if msg.data and msg.data != self.state.robot_description:
            self.state.robot_description = msg.data
            self._publish_state()

    def _poll_status(self) -> None:
        self.state.robot_link_alive = self._probe_robot_link()
        self.state.action_server_ready = self.trajectory.ready()
        self._poll_program_running_service()
        if self._controller_client.service_is_ready():
            future = self._controller_client.call_async(ListControllers.Request())
            future.add_done_callback(self._controller_response)
        self._publish_state()

    def _poll_program_running_service(self) -> None:
        """Refresh program state through the dashboard without commanding motion.

        ``robot_program_running`` remains the normal asynchronous source.  A
        dashboard query is only a fallback for the case where that topic has
        not emitted an initial sample after a driver/External-Control restart.
        """
        if not self._program_client.service_is_ready():
            return
        if self._program_future is not None and not self._program_future.done():
            return
        try:
            self._program_future = self._program_client.call_async(
                IsProgramRunning.Request()
            )
            self._program_future.add_done_callback(self._program_response)
        except Exception:
            self._program_future = None

    def _program_response(self, future) -> None:
        try:
            response = future.result()
            if bool(response.success):
                self.state.program_running = bool(response.program_running)
                self._publish_state()
        except Exception:
            # Keep the last topic/service value.  The normal safety checks
            # will still block motion when no valid state is available.
            pass
        finally:
            self._program_future = None

    def _probe_robot_link(self) -> bool:
        """Check the physical robot TCP link without sending any command.

        This deliberately uses a short TCP connect only.  It prevents a
        disconnected RTDE session from being reported as ``Connected`` when
        ros2_control is still publishing cached joint/status samples.
        """
        for port in (30001, 29999):
            try:
                with socket.create_connection((self.robot_ip, port), timeout=0.15):
                    return True
            except (OSError, TimeoutError):
                continue
        return False

    def _controller_response(self, future) -> None:
        try:
            response = future.result()
            self.state.controller_active = any(
                c.name == "scaled_joint_trajectory_controller" and c.state == "active"
                for c in response.controller
            )
            self._publish_state()
        except Exception as exc:
            self.state.controller_active = False
            self._emit(f"controller status unavailable: {exc}")

    def send(self, trajectory: JointTrajectory, kind: str = "JOG", detail: str = "") -> bool:
        return self.trajectory.send(trajectory, kind=kind, detail=detail)

    def cancel(self) -> bool:
        return self.trajectory.cancel()

    def shutdown(self) -> None:
        self.cancel()
        self.destroy_node()
