from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class RobotState:
    joint_names: List[str] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    joint_stamp: float = 0.0
    program_running: Optional[bool] = None
    robot_mode: Optional[int] = None
    safety_mode: Optional[int] = None
    speed_scaling: Optional[float] = None
    speed_scaling_stamp: float = 0.0
    speed_slider_service_ready: bool = False
    controller_active: bool = False
    action_server_ready: bool = False
    tcp_position: Optional[List[float]] = None
    tcp_frame_id: str = ""
    # Monotonic receipt time of /tcp_pose_broadcaster/pose.  This is kept
    # separate from joint_stamp so the HMI can reject a stale controller TCP
    # pose instead of silently reusing it for the LIVE marker.
    tcp_stamp: float = 0.0
    tcp_quaternion: Optional[List[float]] = None
    tcp_rpy: Optional[List[float]] = None
    robot_description: Optional[str] = None
    goal_active: bool = False
    # Independent TCP reachability probe to the real robot.  ROS 2 control
    # nodes can continue publishing their last sample after an RTDE link is
    # lost, so HMI connectivity must not rely on joint_states freshness alone.
    robot_link_alive: Optional[bool] = None

    def copy(self) -> "RobotState":
        return RobotState(
            joint_names=list(self.joint_names),
            positions=list(self.positions),
            joint_stamp=self.joint_stamp,
            program_running=self.program_running,
            robot_mode=self.robot_mode,
            safety_mode=self.safety_mode,
            speed_scaling=self.speed_scaling,
            speed_scaling_stamp=self.speed_scaling_stamp,
            speed_slider_service_ready=self.speed_slider_service_ready,
            controller_active=self.controller_active,
            action_server_ready=self.action_server_ready,
            tcp_position=None if self.tcp_position is None else list(self.tcp_position),
            tcp_frame_id=self.tcp_frame_id,
            tcp_stamp=self.tcp_stamp,
            tcp_quaternion=None if self.tcp_quaternion is None else list(self.tcp_quaternion),
            tcp_rpy=None if self.tcp_rpy is None else list(self.tcp_rpy),
            robot_description=self.robot_description,
            goal_active=self.goal_active,
            robot_link_alive=self.robot_link_alive,
        )
