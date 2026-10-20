# -*- coding: utf-8 -*-
"""
UR5e Windows 最简：Dashboard 自动上电（到可运行状态为止）

功能：
- 连接 Dashboard Server(29999)
- power on
- brake release
- unlock protective stop（可选）
- 打印 robotmode / safetystatus
"""

import socket
import time

ROBOT_IP = "192.168.0.10"
TRY_UNLOCK_PROTECTIVE_STOP = True


def dash_cmd(robot_ip: str, cmd: str, timeout: float = 2.0) -> str:
    """
    Dashboard Server(29999) 单次短连接发送命令并读取返回。
    注意：连接后通常会先发欢迎语，所以先读一次再发命令。
    """
    with socket.create_connection((robot_ip, 29999), timeout=timeout) as s:
        s.settimeout(timeout)
        # 读欢迎语（可能有）
        try:
            s.recv(4096)
        except Exception:
            pass

        s.sendall((cmd + "\n").encode("utf-8"))
        data = s.recv(4096)
    return data.decode("utf-8", errors="ignore").strip()


def dashboard_power_on(robot_ip: str) -> None:
    print("[DASH]", dash_cmd(robot_ip, "power on"))
    time.sleep(0.5)

    print("[DASH]", dash_cmd(robot_ip, "brake release"))
    time.sleep(0.5)

    if TRY_UNLOCK_PROTECTIVE_STOP:
        print("[DASH]", dash_cmd(robot_ip, "unlock protective stop"))
        time.sleep(0.2)

    print("[DASH]", dash_cmd(robot_ip, "robotmode"))
    print("[DASH]", dash_cmd(robot_ip, "safetystatus"))


def main() -> None:
    print("目标机器人 IP:", ROBOT_IP)
    try:
        dashboard_power_on(ROBOT_IP)
        print("完成：已发送上电/释放抱闸流程（是否完全可控取决于现场安全状态/模式）。")
    except Exception as e:
        print("Dashboard 执行失败：", e)


if __name__ == "__main__":
    main()
