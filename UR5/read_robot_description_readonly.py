#!/usr/bin/env python3
"""Read-only export of the calibrated UR ROS2 robot_description.

The running ur_robot_driver publishes this URDF after applying the
kinematics_params_file.  This script only subscribes and writes the XML;
it does not call any robot control service and does not send motion.
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


OUTPUT = Path(__file__).resolve().parent / "real_robot_description_calibrated.urdf"


class Reader(Node):
    def __init__(self):
        super().__init__("read_robot_description_readonly")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.done = False
        self.sub = self.create_subscription(
            String, "/robot_description", self.callback, qos
        )

    def callback(self, msg: String) -> None:
        OUTPUT.write_text(msg.data, encoding="utf-8")
        print(f"Wrote {len(msg.data)} characters to {OUTPUT}")
        self.done = True


def main() -> None:
    rclpy.init()
    node = Reader()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
