# -*- coding: utf-8 -*-
import socket
import time
import threading
import rtde_receive

# --------- 参数 ----------
ROBOT_IP = "192.168.0.10"

# URScript 发送端口（Secondary Interface）
UR_SCRIPT_PORT = 30002

# 点动参数
FREQ = 50.0
DT = 1.0 / FREQ
ACC = 1.0
V = 0.30
# ------------------------


def dash_cmd(cmd: str, timeout=2.0) -> str:
    """Dashboard(29999) 发命令并读取返回"""
    with socket.create_connection((ROBOT_IP, 29999), timeout=timeout) as s:
        s.settimeout(timeout)
        # 读欢迎语
        try:
            s.recv(4096)
        except Exception:
            pass
        s.sendall((cmd + "\n").encode("utf-8"))
        return s.recv(4096).decode("utf-8", errors="ignore").strip()


def dashboard_startup():
    print("[DASH]", dash_cmd("power on"))
    time.sleep(0.5)
    print("[DASH]", dash_cmd("brake release"))
    time.sleep(0.5)
    print("[DASH]", dash_cmd("unlock protective stop"))
    time.sleep(0.2)
    print("[DASH]", dash_cmd("robotmode"))
    print("[DASH]", dash_cmd("safetystatus"))


def send_urscript(sock: socket.socket, script_line: str):
    if not script_line.endswith("\n"):
        script_line += "\n"
    sock.sendall(script_line.encode("utf-8"))


class URJogger:
    """
    最小点动控制器：
    - jog_start(joint_index, direction): 开始点动（后台线程持续发 speedj）
    - jog_stop(): 停止点动（发 stopj）
    """

    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self.sock = socket.create_connection((robot_ip, UR_SCRIPT_PORT), timeout=2.0)
        self.sock.settimeout(2.0)

        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        # 用于验证运动（可删）
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    def jog_start(self, joint_index: int, direction: int, speed: float = V, acc: float = ACC):
        """
        joint_index: 0~5 => J1~J6
        direction: +1 or -1
        """
        with self._lock:
            # 如果已经在点动，先停掉再换方向/关节
            self._running = False

        # 等前一个线程退出
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)

        joint_speed = [0, 0, 0, 0, 0, 0]
        joint_speed[joint_index] = direction * speed

        def loop():
            with self._lock:
                self._running = True

            next_print = time.time()
            while True:
                with self._lock:
                    if not self._running:
                        break
                # 持续发 speedj
                cmd = f"speedj({joint_speed}, a={acc}, t={DT})"
                send_urscript(self.sock, cmd)

                # 降频打印 actual_q（可删）
                now = time.time()
                if now >= next_print:
                    q = self.rtde_r.getActualQ()
                    print("actual_q:", [round(x, 6) for x in q])
                    next_print = now + 0.2

                time.sleep(DT)

            # 退出时发 stopj
            send_urscript(self.sock, "stopj(2.0)")

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

        print(f"开始点动：J{joint_index+1} dir={direction} speed={speed}")

    def jog_stop(self):
        with self._lock:
            self._running = False
        print("点动停止请求")

    def close(self):
        try:
            self.jog_stop()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    print("目标机器人:", ROBOT_IP)

    # 1) Dashboard：自动上电/释放抱闸
    dashboard_startup()

    jogger = URJogger(ROBOT_IP)

    try:
        # 示例：点动 J6 正转 2 秒
        jogger.jog_start(joint_index=1, direction=+1, speed=2.0)
        time.sleep(5)
        jogger.jog_stop()
        time.sleep(0.5)  # 给 stopj 一点时间

        # 你也可以换成更明显的 J1 测试：
        # jogger.jog_start(joint_index=0, direction=+1, speed=0.10)
        # time.sleep(1.0)
        # jogger.jog_stop()
        # time.sleep(0.5)

    finally:
        jogger.close()

    print("done")


if __name__ == "__main__":
    main()
