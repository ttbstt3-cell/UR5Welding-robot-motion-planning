#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时读取 UR 机器人末端 TCP 相对于基座标系的笛卡尔位姿。

用法：
    python get_tcp_pose.py [--ip ROBOT_IP] [--freq HZ] [--deg]

参数：
    --ip     机器人 IP 地址，默认为 "192.168.0.10"
    --freq   数据刷新频率（Hz），默认为 10 Hz
    --deg    如果添加此选项，输出的姿态角（rx, ry, rz）将转换为度（默认输出弧度）

返回位姿格式：
    [x, y, z, rx, ry, rz]
    - x, y, z: 位置，单位米
    - rx, ry, rz: 旋转向量（轴角），单位弧度（或度，若使用 --deg）

依赖：
    - ur-rtde (已安装)
"""

import argparse
import time
import rtde_receive
import math

def main():
    parser = argparse.ArgumentParser(description="实时读取 UR 机器人末端 TCP 位姿")
    parser.add_argument("--ip", type=str, default="192.168.0.10",
                        help="机器人 IP 地址 (默认: 192.168.0.10)")
    parser.add_argument("--freq", type=float, default=10.0,
                        help="数据刷新频率，单位 Hz (默认: 10)")
    parser.add_argument("--deg", action="store_true",
                        help="将姿态角（rx, ry, rz）转换为度显示（默认输出弧度）")
    args = parser.parse_args()

    ip = args.ip
    freq = args.freq
    dt = 1.0 / freq
    use_deg = args.deg

    print(f"正在连接机器人 {ip} ...")
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ip)
    except Exception as e:
        print(f"连接失败: {e}")
        print("请检查：")
        print("  - 机器人 IP 是否正确")
        print("  - 机器人是否已开机并处于远程模式")
        print("  - 外部控制程序是否运行")
        return

    print("连接成功！开始接收末端 TCP 位姿 (相对于基座标系，按 Ctrl+C 停止)...\n")

    # 表头：位置(m) 和 姿态(rad 或 deg)
    unit_angle = "deg" if use_deg else "rad"
    header = (f"{'x (m)':<12} {'y (m)':<12} {'z (m)':<12} "
              f"{'rx ('+unit_angle+')':<12} {'ry ('+unit_angle+')':<12} {'rz ('+unit_angle+')':<12}")
    print(header)
    print("-" * 80)

    try:
        while True:
            # 获取当前 TCP 位姿（相对于基座标系）
            pose = rtde_r.getActualTCPPose()  # 返回 [x, y, z, rx, ry, rz]，单位：米，弧度

            if use_deg:
                # 将旋转部分从弧度转换为度
                pose_converted = [
                    pose[0], pose[1], pose[2],
                    math.degrees(pose[3]),
                    math.degrees(pose[4]),
                    math.degrees(pose[5])
                ]
            else:
                pose_converted = pose

            # 输出，保留4位小数（位置和角度）
            print("{:<12.4f} {:<12.4f} {:<12.4f} {:<12.4f} {:<12.4f} {:<12.4f}".format(*pose_converted))

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n用户中断，退出程序。")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        print("程序结束。")

if __name__ == "__main__":
    main()


    Pbase=[]