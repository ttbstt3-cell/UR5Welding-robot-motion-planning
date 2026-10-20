"""
UR 机器人 6轴“左右按住持续转”的 speedJ Jog UI（PySide6）
- 按住◀/▶：对应关节持续以 +/- 速度旋转（rad/s）
- 松开：该关节速度归零
- 右侧实时显示关节角度（deg）

依赖：
  pip install pyside6 ur-rtde==1.5.6
"""

import sys
import math
import time
from typing import List, Optional

from PySide6 import QtCore, QtWidgets
import rtde_control
import rtde_receive

# -------- 可调参数 --------
ROBOT_IP = "192.168.0.10"
FREQUENCY = 125.0          # e系列可尝试 500.0（网络/脚本稳定前提下）
DT = 1.0 / FREQUENCY

# 每个关节的“按住转动速度”（rad/s）
JOG_QD = [0.6, 0.6, 0.6, 0.9, 0.9, 1.2]  # J1..J6

ACC_LIMIT = 2.0            # speedJ 的加速度限制（rad/s^2）
FAILSAFE_MS = 400          # 多久没刷新就自动停（防卡死/失联）
# -------------------------


def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


class JointRow(QtWidgets.QWidget):
    """
    一行关节控件：
      名称 | ◀ | slider(只做显示) | ▶ | 数值(°)
    """
    pressed_left = QtCore.Signal(int)   # joint_index
    pressed_right = QtCore.Signal(int)
    released = QtCore.Signal(int)

    def __init__(self, idx: int, name: str, parent=None) -> None:
        super().__init__(parent)
        self.idx = idx

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)

        self.lab_name = QtWidgets.QLabel(name)
        self.lab_name.setFixedWidth(60)

        self.btn_left = QtWidgets.QPushButton("◀")
        self.btn_right = QtWidgets.QPushButton("▶")
        self.btn_left.setAutoRepeat(False)
        self.btn_right.setAutoRepeat(False)
        self.btn_left.setFixedWidth(42)
        self.btn_right.setFixedWidth(42)

        # slider 仅用于“看起来像示教器那样”，不直接控制速度
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        # ✅ 初始给个 ±360° 的视觉范围（0.1°单位），后续会自动扩展
        self.slider.setRange(-3600, 3600)
        self.slider.setValue(0)
        self.slider.setEnabled(False)

        self.lab_val = QtWidgets.QLabel("0.00°")
        self.lab_val.setFixedWidth(90)
        self.lab_val.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)

        layout.addWidget(self.lab_name)
        layout.addWidget(self.btn_left)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.btn_right)
        layout.addWidget(self.lab_val)

        # 按下/松开事件
        self.btn_left.pressed.connect(lambda: self.pressed_left.emit(self.idx))
        self.btn_right.pressed.connect(lambda: self.pressed_right.emit(self.idx))
        self.btn_left.released.connect(lambda: self.released.emit(self.idx))
        self.btn_right.released.connect(lambda: self.released.emit(self.idx))

    def set_deg(self, deg: float) -> None:
        """显示角度，并保证 slider 范围不会越界（自动扩展）"""
        self.lab_val.setText(f"{deg:.2f}°")
        v = int(deg * 10)  # 0.1°单位

        lo, hi = self.slider.minimum(), self.slider.maximum()
        # ✅ 自动扩展范围，避免角度超过 ±360° 时 Qt 报错
        if v < lo:
            self.slider.setMinimum(v - 200)  # 多留 20°
        if v > hi:
            self.slider.setMaximum(v + 200)

        self.slider.setValue(v)


class UrSpeedJJog(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("UR speedJ Jog (◀▶ 按住转动)")

        self.ctrl: Optional[rtde_control.RTDEControlInterface] = None
        self.recv: Optional[rtde_receive.RTDEReceiveInterface] = None

        # 当前 6轴速度命令
        self.qd_cmd: List[float] = [0.0] * 6
        self.last_cmd_ms = int(time.time() * 1000)

        root = QtWidgets.QVBoxLayout(self)

        # 顶部按钮
        top = QtWidgets.QHBoxLayout()
        self.btn_reconnect = QtWidgets.QPushButton("Reconnect")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        top.addWidget(self.btn_reconnect)
        top.addWidget(self.btn_stop)
        top.addStretch(1)
        root.addLayout(top)

        # 关节行
        names = ["机座", "肩部", "肘部", "手腕 1", "手腕 2", "手腕 3"]
        self.rows: List[JointRow] = []
        for i, n in enumerate(names):
            r = JointRow(i, n)
            r.pressed_left.connect(self.on_left)
            r.pressed_right.connect(self.on_right)
            r.released.connect(self.on_release)
            self.rows.append(r)
            root.addWidget(r)

        # 状态
        self.label = QtWidgets.QLabel("Ready")
        root.addWidget(self.label)

        # 定时器：1) 发送 speedJ 2) 刷新关节角显示
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_tick)

        self.btn_reconnect.clicked.connect(self.connect_rtde)
        self.btn_stop.clicked.connect(self.stop_all)

        # 启动先连一次
        self.connect_rtde()

    def _status_suffix(self) -> str:
        if not self.recv:
            return ""
        try:
            rm = self.recv.getRobotMode()
            sm = self.recv.getSafetyMode()
            rs = self.recv.getRuntimeState()
            return f" (robotMode={rm}, safetyMode={sm}, runtimeState={rs})"
        except Exception:
            return ""

    def set_enabled(self, enabled: bool) -> None:
        for r in self.rows:
            r.btn_left.setEnabled(enabled)
            r.btn_right.setEnabled(enabled)

    def connect_rtde(self) -> None:
        self.stop_all()
        self.set_enabled(False)

        try:
            self.recv = rtde_receive.RTDEReceiveInterface(ROBOT_IP, FREQUENCY)
        except Exception as e:
            self.recv = None
            self.ctrl = None
            self.label.setText(f"RTDEReceive 连接失败: {e}")
            return

        try:
            self.ctrl = rtde_control.RTDEControlInterface(ROBOT_IP, FREQUENCY)
        except Exception as e:
            self.ctrl = None
            self.label.setText(f"RTDEControl 连接失败: {e}{self._status_suffix()}")
            return

        ok = False
        try:
            ok = bool(self.ctrl.isConnected()) and bool(self.ctrl.isProgramRunning())
        except Exception:
            ok = False

        if ok:
            self.label.setText(f"Connected & ProgramRunning{self._status_suffix()}")
            self.set_enabled(True)
            if not self.timer.isActive():
                self.timer.start(int(DT * 1000))
        else:
            self.label.setText("控制脚本未运行/未远程/安全停，请检查示教器。" + self._status_suffix())
            self.set_enabled(False)

    # ---- UI 事件：按住左/右 ----
    def on_left(self, joint_idx: int) -> None:
        self._ensure_ready()
        self.qd_cmd[joint_idx] = -abs(JOG_QD[joint_idx])
        self.last_cmd_ms = int(time.time() * 1000)

    def on_right(self, joint_idx: int) -> None:
        self._ensure_ready()
        self.qd_cmd[joint_idx] = abs(JOG_QD[joint_idx])
        self.last_cmd_ms = int(time.time() * 1000)

    def on_release(self, joint_idx: int) -> None:
        self.qd_cmd[joint_idx] = 0.0
        self.last_cmd_ms = int(time.time() * 1000)

    def _ensure_ready(self) -> None:
        if not self.ctrl:
            self.label.setText("未连接，请点 Reconnect。")
            return
        try:
            if (not self.ctrl.isConnected()) or (not self.ctrl.isProgramRunning()):
                self.label.setText("脚本未运行/断线，尝试重连…")
                self.connect_rtde()
        except Exception:
            self.connect_rtde()

    def stop_all(self) -> None:
        self.qd_cmd = [0.0] * 6
        self.last_cmd_ms = int(time.time() * 1000)
        try:
            if self.ctrl:
                self.ctrl.speedStop()
        except Exception:
            pass

    def on_tick(self) -> None:
        # 刷关节角显示
        if self.recv:
            try:
                q = self.recv.getActualQ()  # rad
                for i in range(6):
                    self.rows[i].set_deg(rad2deg(q[i]))
            except Exception:
                pass

        if not self.ctrl:
            return

        # ✅ 修复“按住一会就停”：
        # 只要当前仍有非零速度命令（说明还在按住），就持续刷新 last_cmd_ms
        if any(abs(v) > 1e-9 for v in self.qd_cmd):
            self.last_cmd_ms = int(time.time() * 1000)

        # failsafe：太久没刷新（比如UI卡住）就归零
        now_ms = int(time.time() * 1000)
        if (now_ms - self.last_cmd_ms) > FAILSAFE_MS:
            self.qd_cmd = [0.0] * 6

        # 状态检查
        try:
            if (not self.ctrl.isConnected()) or (not self.ctrl.isProgramRunning()):
                self.stop_all()
                self.set_enabled(False)
                self.label.setText("断线/脚本停止，已自动停。" + self._status_suffix())
                return
        except Exception:
            self.stop_all()
            self.set_enabled(False)
            self.label.setText("状态读取异常，已自动停。")
            return

        # 发 speedJ
        try:
            self.ctrl.speedJ(self.qd_cmd, ACC_LIMIT, DT)
        except Exception as e:
            self.label.setText(f"speedJ error: {e}")

    def closeEvent(self, event) -> None:
        self.stop_all()
        try:
            if self.ctrl:
                self.ctrl.stopScript()
        except Exception:
            pass
        event.accept()


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    w = UrSpeedJJog()
    w.resize(900, 260)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
