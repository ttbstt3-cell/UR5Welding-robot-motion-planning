"""Bounded ROS status/service reads only. No control/goal/write interfaces."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import IsProgramRunning


def run(output):
    rclpy.init()
    node = Node("ur5_readonly_execution_diagnostic")
    result = dict(timestamp=datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
                  robot_motion="NO", read_only=True, topics={}, services={}, publishers={})
    subscriptions = []
    topics = [
        (Bool, "/io_and_status_controller/robot_program_running", "data"),
        (RobotMode, "/io_and_status_controller/robot_mode", "mode"),
        (SafetyMode, "/io_and_status_controller/safety_mode", "mode"),
        (Float64, "/speed_scaling_state_broadcaster/speed_scaling", "data"),
    ]
    started = time.monotonic()
    for typ, topic, field in topics:
        for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            key = topic + ":" + durability.name
            result["topics"][key] = dict(samples=0, last_value=None)
            def receive(msg, key=key, field=field):
                item = result["topics"][key]
                item.update(samples=item["samples"] + 1, last_value=getattr(msg, field),
                            received_after_sec=time.monotonic() - started)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=durability)
            subscriptions.append(node.create_subscription(typ, topic, receive, qos))
    result["topics"]["/joint_states"] = dict(samples=0)
    def joints(msg):
        item = result["topics"]["/joint_states"]
        item.update(samples=item["samples"] + 1, names=list(msg.name), positions=list(msg.position))
    subscriptions.append(node.create_subscription(JointState, "/joint_states", joints, 10))
    parameter_request = GetParameters.Request()
    parameter_request.names = ["joint_controller_active", "headless_mode"]
    requests = [
        ("controllers", node.create_client(ListControllers, "/controller_manager/list_controllers"), ListControllers.Request()),
        ("program_running", node.create_client(IsProgramRunning, "/dashboard_client/program_running"), IsProgramRunning.Request()),
        ("stopper_parameters", node.create_client(GetParameters, "/controller_stopper/get_parameters"), parameter_request),
    ]
    pending = {}
    try:
        while time.monotonic() - started < 9:
            for name, client, request in requests:
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(request)
            rclpy.spin_once(node, timeout_sec=.05)
        for name, future in pending.items():
            if not future.done():
                result["services"][name] = dict(status="TIMEOUT")
                continue
            try:
                response = future.result()
                if name == "controllers":
                    result["services"][name] = [dict(name=c.name, state=c.state, type=c.type) for c in response.controller]
                elif name == "program_running":
                    result["services"][name] = dict(success=response.success, program_running=response.program_running, answer=response.answer)
                else:
                    result["services"][name] = {key: dict(type=value.type, bool_value=value.bool_value)
                        for key, value in zip(parameter_request.names, response.values)}
            except Exception as error:
                result["services"][name] = dict(status="ERROR", message=str(error))
        for _, topic, _ in topics:
            result["publishers"][topic] = [dict(node=p.node_name, namespace=p.node_namespace,
                reliability=p.qos_profile.reliability.name, durability=p.qos_profile.durability.name)
                for p in node.get_publishers_info_by_topic(topic)]
        for name, _, _ in requests:
            result["services"].setdefault(name, dict(status="UNAVAILABLE"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    run(args.output.resolve())
