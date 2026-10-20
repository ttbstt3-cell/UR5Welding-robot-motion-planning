"""No-robot tests: fake service futures + real GUI/VTK with synthetic feedback.

No ROS node starts and no hardware service/action is called. The action client
is replaced by a local fake; GUI speed signals are observed with ROS disabled.
"""
import copy
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch
import warnings

os.environ.setdefault("QT_QPA_PLATFORM", "xcb" if "--vtk" in sys.argv else "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5.QtCore import QThread, Qt
from PyQt5.QtWidgets import QApplication, QMessageBox
from control_msgs.action import FollowJointTrajectory
from gui.main_window import CONFIG, MainWindow, RosWorker
from ros.state_monitor import RobotState
from ros.speed_slider import SpeedSliderClient
from ros.trajectory_client import MotionState, TrajectoryClient
from trajectory.execution_feedback import decode_feedback
from trajectory.result_store import read_index
from trajectory.trajectory_utils import trajectory_to_ros_message
from safety.execution_preparation import file_hash


class Future:
    def __init__(self): self.callback = None; self.response = None
    def add_done_callback(self, callback): self.callback = callback
    def result(self): return self.response
    def complete(self, value): self.response = value; self.callback(self)


class FakeService:
    def __init__(self): self.calls = []; self.available = True
    def service_is_ready(self): return self.available
    def call_async(self, request):
        self.calls.append(request.speed_slider_fraction)
        self.future = Future(); return self.future


class FakeNode:
    def __init__(self): self.service = FakeService()
    def create_client(self, *_): return self.service
    def create_timer(self, *_): return object()


def ready(q=None):
    return RobotState(positions=list(q or [0.]*6), joint_stamp=time.monotonic(),
        program_running=True, robot_mode=7, safety_mode=1, controller_active=True,
        action_server_ready=True, speed_scaling=25., speed_scaling_stamp=time.monotonic(),
        speed_slider_service_ready=True, robot_link_alive=True)


def service_tests():
    now = [time.monotonic()]; state = ready(); state.joint_stamp = state.speed_scaling_stamp = now[0]
    node = FakeNode(); done = []
    client = SpeedSliderClient(node, lambda: state, lambda *args: done.append(args), lambda: now[0])
    assert not node.service.calls, "Startup wrote a speed"
    rejected = []
    for p in (-1, 0, 100.01, 200, float("nan"), float("inf")):
        assert not client.request(p)
    assert not node.service.calls
    for p in (1., 20., 100.):
        assert client.request(p)
        assert node.service.calls[-1] == p/100.
        assert not client.request(50), "Overlapped speed commands"
        node.service.future.complete(SimpleNamespace(success=True))
    assert state.speed_scaling == 25., "ACK forged an actual readback"
    assert client.request(40)
    now[0] += 4; client._check_timeout()
    assert client.busy and "UNKNOWN" in done[-1][2]
    assert not client.request(50)
    node.service.future.complete(SimpleNamespace(success=True))
    assert not client.busy and done[-1][0] and "延迟" in done[-1][2]
    state.joint_stamp = state.speed_scaling_stamp = now[0]
    for field, value in (("robot_link_alive", False), ("robot_mode", 3),
        ("program_running", False), ("safety_mode", 3), ("controller_active", False),
        ("speed_slider_service_ready", False), ("joint_stamp", now[0]-2),
        ("speed_scaling_stamp", now[0]-2), ("speed_scaling", float("nan"))):
        original = getattr(state, field); setattr(state, field, value)
        calls = len(node.service.calls); assert not client.request(50)
        assert calls == len(node.service.calls); rejected.append(field)
        setattr(state, field, original)
    node.service.available = False; assert not client.request(50)
    return dict(pass_all=True, zero_hardware_calls=True, fake_service_only=True,
        startup_writes=0, endpoints_1_100_percent_pass=True, invalid_percent_rejected=True,
        overlapping_commands_rejected=True, timeout_remains_unknown_until_late_result=True,
        ack_never_replaces_actual_feedback=True, stale_faults_rejected=rejected)


def no_tcp_reads(self): return None


def action_feedback_tests(trajectory):
    class Action:
        def __init__(self, *_): self.sent = []; self.future = None
        def server_is_ready(self): return True
        def send_goal_async(self, goal, feedback_callback=None):
            self.sent.append(goal); self.feedback_callback = feedback_callback
            self.future = Future(); return self.future
    active = []; done = []; feedback = []
    with patch("ros.trajectory_client.ActionClient", Action):
        client = TrajectoryClient(FakeNode(), "/TEST_ONLY", active.append,
            lambda *args: done.append(args), on_feedback=feedback.append)
    message = trajectory_to_ros_message(trajectory)
    # Compare all ROS fields, not CDR alignment bytes (padding is not payload).
    original = copy.deepcopy(message)
    assert client.send(message, kind="TRAJ")
    generation = client._feedback_generation
    result_future = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result_future)
    client.client.future.complete(handle)
    f = FollowJointTrajectory.Feedback(); f.joint_names = list(trajectory.joint_names)
    f.desired.positions = trajectory.positions[0].tolist(); f.actual.positions = trajectory.positions[0].tolist()
    wrapped = SimpleNamespace(feedback=f)
    client._feedback_callback(wrapped, generation-1, "TRAJ"); assert not feedback
    client.client.feedback_callback(wrapped); assert len(feedback) == 1
    f.joint_names[0] = "WRONG"; client.client.feedback_callback(wrapped); assert len(feedback) == 1
    f.joint_names = list(trajectory.joint_names)
    result_future.complete(SimpleNamespace(status=4, result=SimpleNamespace(error_code=0)))
    client.client.feedback_callback(wrapped); assert len(feedback) == 1
    assert not client.busy and done[-1][0]
    assert message == original
    assert client.client.sent[0].trajectory == original
    # Direct worker gate with an isolated fake service (no ROS node).
    state = ready(); node = FakeNode(); results = []
    slider = SpeedSliderClient(node, lambda: state, lambda *args: results.append(args))
    worker = RosWorker("/TEST_ONLY")
    worker.node = SimpleNamespace(trajectory=SimpleNamespace(busy=True,
        motion_state=MotionState.SENDING, current_goal_label="TRAJ #001"), speed_slider=slider)
    for mode in (MotionState.SENDING, MotionState.CANCELING, MotionState.FAULT_UNKNOWN):
        worker.node.trajectory.motion_state = mode
        worker._set_speed_percent(50)
        assert not node.service.calls
    worker.node.trajectory.motion_state = MotionState.EXECUTING
    worker.node.trajectory.current_goal_label = "JOG #001"
    worker._set_speed_percent(50); assert not node.service.calls
    worker.node.trajectory.current_goal_label = "TRAJ #001"
    worker._set_speed_percent(50); assert node.service.calls == [.5]
    node.service.future.complete(SimpleNamespace(success=True))
    return dict(pass_all=True, fake_action_only=True, new_feedback_hook_does_not_modify_message=True,
        old_goal_invalid_joint_and_post_terminal_feedback_ignored=True,
        worker_blocks_sending_canceling_unknown_and_jog_speed_changes=True,
        real_network_calls=0)


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    warnings.filterwarnings("ignore", message="Glyph.*missing from current font")
    result = dict(service=service_tests())
    root = Path(__file__).resolve().parents[2]
    entry = read_index(root / "规划文件/当前轨迹")["prepare"]
    source = Path(entry["path"]); before = file_hash(source); config_before = CONFIG.read_bytes()
    app = QApplication.instance() or QApplication([])
    with patch.object(QThread, "start", lambda self: None), \
         patch.object(MainWindow, "_auto_read_robot_tcp", no_tcp_reads), \
         patch.object(MainWindow, "showMaximized", lambda self: None):
        win = MainWindow()
    win.robot_tcp_auto_timer.stop()
    win.setAttribute(Qt.WA_ShowWithoutActivating)
    win.setWindowTitle("UR5 SPEED/LIVE UI TEST — synthetic feedback, ROS disabled")
    sends = []; speed_requests = []
    win.worker.request_trajectory.connect(sends.append)
    win.worker.request_speed_percent.connect(speed_requests.append)
    try:
        win._adopt_trajectory_file(source)
        traj = win.loaded_trajectory; q0 = traj.positions[0].tolist()
        result["action_feedback"] = action_feedback_tests(traj)
        state = ready(q0); win._on_state(state); win._refresh_state_display()
        win.real_speed_spin.setValue(60); win.real_speed_slider.setValue(40)
        assert not speed_requests, "Dragging changed hardware without apply"
        win.real_speed_apply_button.click()
        assert speed_requests == [40.] and win.real_speed_pending
        win._on_speed_request_done(True, 40., "FAKE ACK")
        assert win.state.speed_scaling == 25. and "25.0%" in win.real_speed_actual_label.text()
        # Start offline preview, then simulate action lifecycle without sending.
        win._start_preview(); assert win.preview_active and win.robot_view.preview_active
        win.execution_trajectory = traj
        win.trajectory_client = SimpleNamespace(busy=True, motion_state=MotionState.EXECUTING,
            current_goal_label="TRAJ #001", cancel_timed_out=False)
        win._on_motion("EXECUTING", True, "TRAJ #001"); win._on_goal_active(True)
        assert not win.preview_active and not win.robot_view.preview_active
        assert not win.preview_timer.isActive()
        frames = []
        for i in (0, 1, 2, 5, 10):
            q = traj.positions[i].tolist(); state = ready(q)
            win._on_state(state); win._refresh_state_display()
            assert np.array_equal(win.robot_view._live_q, q)
            assert not win.robot_view.preview_active
            if hasattr(win.robot_view, "_refresh"):
                win.robot_view._refresh()
            tool_check = None
            if hasattr(win.robot_view, "_gun_actor"):
                view = win.robot_view
                transforms = view.model.link_transforms_in_base(q)
                matrix = view._gun_actor.GetUserMatrix()
                gun = np.array([[matrix.GetElement(r,c) for c in range(4)] for r in range(4)])
                assert np.allclose(gun, transforms["flange"] @ view._gun_actor_mount_transform, atol=1e-12)
                assert np.allclose(view._wire_line_source.GetPoint2(), view._live_tcp_fk, atol=1e-10)
                tool_check = "VTK gun flange attachment and wire live-TCP endpoint PASS"
            assert not win.preview_play_button.isEnabled() and not win.preview_slider.isEnabled()
            win._set_preview_pose(100); win._start_preview()
            assert not win.robot_view.preview_active and np.array_equal(win.robot_view._live_q, q)
            frames.append(dict(index=i, actual_q=q, actual_vtk_tool_update=tool_check))
        # Matching controller feedback, standard order conversion, no modulo.
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = list(reversed(traj.joint_names))
        actual = np.asarray(state.positions)
        reference = actual + .0001
        fb.desired.positions = list(reference[::-1]); fb.actual.positions = list(actual[::-1])
        endpoint = float(traj.time_from_start[11])
        fb.desired.time_from_start.sec = int(endpoint)
        fb.desired.time_from_start.nanosec = int((endpoint-int(endpoint))*1e9)
        record = decode_feedback(SimpleNamespace(feedback=fb), "TRAJ #001", time.monotonic())
        assert np.allclose(record["reference_q"], reference)
        win._on_trajectory_feedback(record)
        win.execution_started_monotonic = time.monotonic() - 10*traj.duration
        if win.execution_log: win.execution_log[-1]["monotonic"] -= 1.
        win._record_execution_sample()
        assert win.execution_log and np.allclose(win.execution_log[-1]["joint_error"], -.0001)
        assert "段下界" in win.execution_progress_label.text() and "100.0%" not in win.execution_progress_label.text()
        # Wrong goal feedback must not replace the reference.
        bad = dict(record, goal_label="TRAJ #OLD"); win._on_trajectory_feedback(bad)
        assert win.execution_feedback is record
        win.execution_feedback["received"] -= 2.; win.execution_log[-1]["monotonic"] -= 1
        win._record_execution_sample()
        assert win.execution_log[-1]["joint_error"] is None and "UNKNOWN" in win.execution_progress_label.text()
        win.state.joint_stamp -= 2.; win.state.speed_scaling_stamp -= 2.
        win._refresh_state_display()
        assert "UNKNOWN" in win.live_execution_label.text() and not win.real_speed_apply_button.isEnabled()
        # Runtime apply requires explicit second confirmation; cancel emits no command.
        win.state = ready(traj.positions[10].tolist()); win._update_buttons()
        with patch.object(QMessageBox, "question", return_value=QMessageBox.No):
            win.real_speed_apply_button.click()
        assert speed_requests == [40.]
        with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
            win.real_speed_apply_button.click()
        assert speed_requests == [40., 40.]
        win._on_speed_request_done(False, 40., "测试拒绝")
        win._on_motion("CANCELING", True, "TRAJ #001")
        win._on_goal_done(False, "trajectory goal cancel requested")
        assert win.execution_trajectory is traj and "待确认" in win.execution_progress_label.text()
        # Robot motion/message data were never scaled or sent.
        assert not sends and file_hash(source) == before and CONFIG.read_bytes() == config_before
        win.showNormal(); win.resize(1920, 1080); win.tabs.setCurrentIndex(win.trajectory_tab_index)
        app.processEvents()
        win.trajectory_scroll.ensureWidgetVisible(win.live_execution_label)
        app.processEvents(); win.grab().save(str(output / "speed_live_ui.png"))
        win.real_speed_apply_button.parentWidget().grab().save(str(output / "speed_live_panel.png"))
        result.update(pass_all=True, view=type(win.robot_view).__name__, synthetic_feedback_frames=frames,
            live_model_not_preview=True, animation_uses_measured_joints=True,
            preview_controls_blocked_during_motion=True, percent_changes_without_apply_send_nothing=True,
            actual_feedback_not_forged_by_ack=True, unknown_stale_feedback_not_wall_clock_fabricated=True,
            runtime_speed_change_requires_manual_confirmation=True, fake_speed_signals=speed_requests,
            robot_trajectory_signals=0, ROS_node_started=False, original_trajectory_and_config_unchanged=True)
        (output / "speed_live_tests.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        win.goal_active = False; win.execution_trajectory = None
        win.trajectory_client = None; win.close()


if __name__ == "__main__": run(Path(sys.argv[1]).resolve())
