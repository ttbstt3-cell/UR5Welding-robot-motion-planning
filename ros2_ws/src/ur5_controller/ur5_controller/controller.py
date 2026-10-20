#!/usr/bin/env python3
"""ROS 2 JointTrajectory controller for the CoppeliaSim UR5.

The node accepts ``trajectory_msgs/JointTrajectory`` on ``/joint_trajectory``
and linearly interpolates its points at 50 Hz. The interpolated positions are
sent to the six simulated force-controlled joints through CoppeliaSim's ZMQ
Remote API. CoppeliaSim itself remains feedback-only and publishes
``/joint_states`` through its embedded simROS2 script.

By default a ten-point two-second demo trajectory is published once after
startup. Set ``send_demo:=false`` when MATLAB or another planner is the
trajectory source.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# CoppeliaSim ships its Python ZMQ client outside the normal site-packages
# path. Add that path locally so ``ros2 run`` works without PYTHONPATH.
_ZMQ_CLIENT_PATH = '/opt/coppeliaSim/programming/zmqRemoteApi/clients/python/src'
if os.path.isdir(_ZMQ_CLIENT_PATH) and _ZMQ_CLIENT_PATH not in sys.path:
    sys.path.insert(0, _ZMQ_CLIENT_PATH)

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    RemoteAPIClient = None


class UR5TrajectoryController(Node):
    """Receive, interpolate, and execute UR5 joint trajectories."""

    JOINT_NAMES = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
    ]

    DEMO_TARGET = [
        0.1745,
        -0.5236,
        0.7854,
        -0.5236,
        0.3491,
        0.1745,
    ]

    def __init__(self):
        super().__init__('ur5_trajectory_controller')

        self.declare_parameter('send_demo', True)

        # The publisher is used for the self-contained demo. External planners
        # (MATLAB/Bi-RRT*) publish to this same standard trajectory interface.
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10,
        )
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self._trajectory_callback,
            10,
        )
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._state_callback,
            10,
        )

        self._remote_client = None
        self._sim = None
        self._joint_handles = []
        self._remote_error_reported = False
        self._last_state_log = 0.0

        self._trajectory_plan = []
        self._trajectory_start = None
        self._trajectory_active = False
        self._trajectory_complete_reported = False

        self._connect_coppeliasim()
        self._update_timer = self.create_timer(0.02, self._update_trajectory)

        self._demo_timer = None
        if bool(self.get_parameter('send_demo').value):
            self._demo_timer = self.create_timer(1.0, self._send_demo_trajectory)

        self.get_logger().info(
            'UR5 JointTrajectory controller started '
            '(topic: /joint_trajectory)'
        )

    def _connect_coppeliasim(self):
        if RemoteAPIClient is None:
            self.get_logger().warning(
                'CoppeliaSim ZMQ client unavailable; trajectory will be received '
                'but cannot be executed'
            )
            return

        try:
            self._remote_client = RemoteAPIClient('127.0.0.1', 23000)
            self._sim = self._remote_client.require('sim')
            ur5 = self._sim.getObject('/UR5')
            self._joint_handles = list(
                self._sim.getObjectsInTree(ur5, self._sim.sceneobject_joint, 0)
            )
            if len(self._joint_handles) != len(self.JOINT_NAMES):
                raise RuntimeError(
                    f'expected 6 UR5 joints, got {len(self._joint_handles)}'
                )
            self.get_logger().info(
                f'Connected to CoppeliaSim ZMQ Remote API '
                f'({len(self._joint_handles)} joints)'
            )
        except Exception as exc:
            self._remote_client = None
            self._sim = None
            self._joint_handles = []
            self.get_logger().warning(f'CoppeliaSim bridge unavailable: {exc}')

    @staticmethod
    def _duration_to_seconds(duration):
        return float(duration.sec) + float(duration.nanosec) * 1.0e-9

    @staticmethod
    def _seconds_to_duration(seconds):
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        result = JointTrajectoryPoint()
        result.time_from_start.sec = sec
        result.time_from_start.nanosec = nanosec
        return result.time_from_start

    def _send_demo_trajectory(self):
        if self._demo_timer is not None:
            self._demo_timer.cancel()

        msg = JointTrajectory()
        msg.joint_names = list(self.JOINT_NAMES)

        # Ten points, including the zero start and the target at t=2 s.
        for index in range(10):
            alpha = index / 9.0
            point = JointTrajectoryPoint()
            point.positions = [alpha * q for q in self.DEMO_TARGET]
            point.time_from_start = self._seconds_to_duration(2.0 * alpha)
            msg.points.append(point)

        self.trajectory_pub.publish(msg)
        self.get_logger().info(
            'Demo trajectory sent: 10 points, duration 2.00 s'
        )

    def _trajectory_callback(self, msg):
        if len(msg.joint_names) != len(self.JOINT_NAMES):
            self.get_logger().warning(
                f'Rejected trajectory: expected 6 joint names, '
                f'got {len(msg.joint_names)}'
            )
            return

        name_to_index = {name: index for index, name in enumerate(msg.joint_names)}
        if set(name_to_index) != set(self.JOINT_NAMES):
            self.get_logger().warning(
                'Rejected trajectory: joint names do not match the UR5 model'
            )
            return

        plan = []
        previous_time = -1.0
        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                self.get_logger().warning(
                    'Rejected trajectory: every point must contain 6 positions'
                )
                return

            point_time = self._duration_to_seconds(point.time_from_start)
            if point_time < 0.0 or point_time <= previous_time:
                self.get_logger().warning(
                    'Rejected trajectory: time_from_start must be strictly increasing'
                )
                return

            positions = [
                float(point.positions[name_to_index[name]])
                for name in self.JOINT_NAMES
            ]
            plan.append((point_time, positions))
            previous_time = point_time

        if not plan:
            self.get_logger().warning('Rejected trajectory: no points received')
            return

        self._trajectory_plan = plan
        self._trajectory_start = time.monotonic()
        self._trajectory_active = True
        self._trajectory_complete_reported = False
        self.get_logger().info(
            f'Accepted trajectory: {len(plan)} points, '
            f'duration {plan[-1][0]:.3f} s'
        )

    def _state_callback(self, msg):
        now = time.monotonic()
        if now - self._last_state_log < 1.0 or len(msg.position) < 6:
            return
        self._last_state_log = now
        degrees = [
            round(float(q) * 180.0 / math.pi, 2)
            for q in msg.position[:6]
        ]
        self.get_logger().info(f'Joint state (deg): {degrees}')

    def _apply_positions(self, positions):
        if self._sim is None or len(self._joint_handles) != 6:
            return
        try:
            # The UR5 joints are force-controlled in this scene. A direct
            # setJointPosition is overwritten by dynamics, so target position
            # is the correct actuator-side API.
            for handle, position in zip(self._joint_handles, positions):
                self._sim.setJointTargetPosition(handle, float(position))
        except Exception as exc:
            if not self._remote_error_reported:
                self._remote_error_reported = True
                self.get_logger().warning(
                    f'Failed to apply CoppeliaSim trajectory point: {exc}'
                )

    def _update_trajectory(self):
        if not self._trajectory_active or not self._trajectory_plan:
            return

        elapsed = time.monotonic() - self._trajectory_start
        first_time, first_position = self._trajectory_plan[0]

        if elapsed <= first_time:
            positions = first_position
        elif elapsed >= self._trajectory_plan[-1][0]:
            positions = self._trajectory_plan[-1][1]
            self._trajectory_active = False
            if not self._trajectory_complete_reported:
                self._trajectory_complete_reported = True
                self.get_logger().info('Trajectory execution completed')
        else:
            positions = self._trajectory_plan[-1][1]
            for (time_a, position_a), (time_b, position_b) in zip(
                self._trajectory_plan[:-1], self._trajectory_plan[1:]
            ):
                if elapsed <= time_b:
                    alpha = (elapsed - time_a) / (time_b - time_a)
                    positions = [
                        a + alpha * (b - a)
                        for a, b in zip(position_a, position_b)
                    ]
                    break

        self._apply_positions(positions)


def main(args=None):
    rclpy.init(args=args)
    node = UR5TrajectoryController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A SIGTERM can invalidate the rcl context before spin() returns.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
