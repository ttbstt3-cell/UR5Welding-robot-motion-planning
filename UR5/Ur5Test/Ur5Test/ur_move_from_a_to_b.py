#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5循环运动控制(socket)
"""

import socket
import struct
import time
import threading
import sys
import os
import select

# ==================== 配置区域 ====================
ROBOT_IP = "192.168.0.10"
ROBOT_PORT = 30002          # 控制端口
REALTIME_PORT = 30003       # 实时数据端口

TCP_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_SPEED = 0.25
JOINT_ACCEL = 0.35
LINEAR_SPEED = 0.05
LINEAR_ACCEL = 0.25


INITIAL_POINT = [0.5917,-0.0276,0.4843,0.1229,2.3172,0.1548] #安全点笛卡尔位姿
POINT_A = [0.7302,-0.2144,0.4153,0.1230,2.3172,0.1548] #a点笛卡尔位姿
POINT_B = [0.7302,-0.0276,0.4153,0.1230,2.3172,0.1548] #b点笛卡尔位姿

# ================================================



class RealtimeClient:
    """30003实时数据客户端（与之前相同）"""
    def __init__(self):
        self.socket = None
        self.connected = False
        self.actual_tcp = [0.0] * 6
        self.lock = threading.Lock()

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((ROBOT_IP, REALTIME_PORT))
            self.connected = True

            self.thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.thread.start()

            for i in range(10):
                time.sleep(0.1)
                if self.get_actual_tcp() != [0.0] * 6:
                    print("✓ 实时数据连接成功")
                    return True
            print("⚠ 连接成功但未收到数据")
            return True
        except Exception as e:
            print(f"✗ 实时数据连接失败: {e}")
            return False

    def _receive_loop(self):
        while self.connected:
            try:
                data = self.socket.recv(1108)
                if len(data) >= 1108:
                    self._parse_packet(data)
            except:
                pass

    def _parse_packet(self, data):
        try:
            tcp = struct.unpack_from('!dddddd', data, 444)
            with self.lock:
                self.actual_tcp = list(tcp)
        except:
            pass

    def get_actual_tcp(self):
        with self.lock:
            return self.actual_tcp.copy()

    def get_actual_flange(self):
        tcp = self.get_actual_tcp()
        return [tcp[i] - TCP_OFFSET[i] for i in range(6)]

    def disconnect(self):
        self.connected = False
        if self.socket:
            self.socket.close()


class URController:
    """30002端口控制（与之前相同）"""
    def __init__(self):
        self.socket = None

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((ROBOT_IP, ROBOT_PORT))
            print("✓ 控制端口连接成功")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    def disconnect(self):
        if self.socket:
            self.socket.close()

    def send(self, cmd):
        self.socket.sendall((cmd + "\n").encode())

    def movej(self, pose):
        self.send(f"movej(p{pose}, a={JOINT_ACCEL}, v={JOINT_SPEED})")

    def movel(self, pose):
        self.send(f"movel(p{pose}, a={LINEAR_ACCEL}, v={LINEAR_SPEED})")


def print_actual_pose(realtime, name="当前"):
    flange = realtime.get_actual_flange()
    tcp = realtime.get_actual_tcp()
    print(f"\n【{name}实际位姿】")
    print(f"  法兰盘: X={flange[0]:.4f}, Y={flange[1]:.4f}, Z={flange[2]:.4f}, "
          f"RX={flange[3]:.4f}, RY={flange[4]:.4f}, RZ={flange[5]:.4f}")
    print(f"  TCP:    X={tcp[0]:.4f}, Y={tcp[1]:.4f}, Z={tcp[2]:.4f}, "
          f"RX={tcp[3]:.4f}, RY={tcp[4]:.4f}, RZ={tcp[5]:.4f}")


def print_plan_pose(pose, name):
    tcp = [pose[i] + TCP_OFFSET[i] for i in range(6)]
    print(f"\n【{name}计划位姿】")
    print(f"  法兰盘: {pose}")
    print(f"  TCP:    {tcp}")


def clear_input_buffer():
    """清空标准输入缓冲区（跨平台）"""
    if sys.platform == "win32":
        try:
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getch()
        except ImportError:
            pass
    else:
        try:
            while select.select([sys.stdin], [], [], 0)[0]:
                os.read(sys.stdin.fileno(), 1)
        except Exception:
            pass


def wait_for_enter(prompt):
    """
    非阻塞等待用户按下回车，同时响应 Ctrl+C。
    跨平台实现（Windows 用 msvcrt，Unix-like 用 select + tty）。
    """
    print(prompt, end='', flush=True)

    # 清空之前可能残留的输入
    clear_input_buffer()

    if sys.platform == "win32":
        import msvcrt
        while True:
            # 检查是否有按键
            if msvcrt.kbhit():
                key = msvcrt.getch()
                # 回车键
                if key in (b'\r', b'\n'):
                    print()  # 换行
                    return
                # Ctrl+C 手动抛出异常
                if key == b'\x03':
                    raise KeyboardInterrupt
            # 短暂休眠，让 Python 有机会处理信号
            time.sleep(0.05)
    else:
        # Unix-like 系统（Linux/macOS）
        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # 设置为原始模式，以便立即读取单个字符
            tty.setraw(fd)
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == '\n' or ch == '\r':
                        print()  # 换行（原始模式下需要手动输出）
                        return
                    if ch == '\x03':   # Ctrl+C
                        raise KeyboardInterrupt
                time.sleep(0.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def wait_confirm(step, realtime, target_pose):
    """等待确认，显示位姿并使用非阻塞输入等待"""
    print_actual_pose(realtime, "当前")
    print_plan_pose(target_pose, f"目标({step})")

    wait_for_enter(f"\n>>> 【{step}】按 Enter 执行...")
    print("  执行中...")


def main():
    print("=" * 70)
    print("UR5循环运动控制（socket + 实时位姿）")
    print(f"机械臂: {ROBOT_IP}")
    print("=" * 70)

    realtime = RealtimeClient()
    if not realtime.connect():
        print("无法获取实际位姿，继续运行")

    ur = URController()
    if not ur.connect():
        realtime.disconnect()
        return

    print("\n计划位姿：")
    print(f"  初始点: {INITIAL_POINT}")
    print(f"  A点:    {POINT_A}")
    print(f"  B点:    {POINT_B}")
    print("\n按 Ctrl+C 立即停止")

    try:
        # 开机初始化
        print(f"\n{'=' * 50}")
        print("开机初始化")
        print(f"{'=' * 50}")

        wait_confirm("开机回初始点", realtime, INITIAL_POINT)
        ur.movej(INITIAL_POINT)
        time.sleep(2)
        print("✓ 已到达初始点")
        print("✓ 初始化完成，进入循环模式")

        # 循环运动
        cycle = 1
        while True:
            print(f"\n{'=' * 50}")
            print(f"第 {cycle} 轮循环")
            print(f"{'=' * 50}")

            wait_confirm("初始点 → A点", realtime, POINT_A)
            ur.movej(POINT_A)
            time.sleep(2)

            wait_confirm("A点 → B点（直线）", realtime, POINT_B)
            ur.movel(POINT_B)
            time.sleep(2)

            wait_confirm("B点 → 初始点", realtime, INITIAL_POINT)
            ur.movej(INITIAL_POINT)
            time.sleep(2)

            print(f"✓ 第 {cycle} 轮完成")
            cycle += 1

    except KeyboardInterrupt:
        print("\n\n! 用户停止，正在断开连接...")
    finally:
        ur.disconnect()
        realtime.disconnect()
        print("程序已安全退出。")


if __name__ == "__main__":
    main()