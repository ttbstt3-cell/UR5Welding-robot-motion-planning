import time
import serial
import rtde_control
import argparse
import sys
import importlib

ROBOT_IP = "192.168.0.10"

FREQ = 125.0
DT = 1.0 / FREQ
ACC_LIMIT = 2.0

JOG_QD = [0.6, 0.6, 0.6, 0.9, 0.9, 1.2]

PORT = "COM4"
BAUD = 9600  # 强烈建议后面改 115200

FAILSAFE_MS = 300

MODE_DISABLE = 0
MODE_CAR = 1
MODE_ARM = 2


def parse_line(line: str):
    # 期望：M,mode,a0,a1,a2,a3,a4,a5
    parts = line.strip().split(",")
    if len(parts) != 8 or parts[0] != "M":
        return None
    try:
        mode = int(parts[1])
        axes = [int(x) for x in parts[2:]]
    except ValueError:
        return None
    axes = [(-1 if x < 0 else (1 if x > 0 else 0)) for x in axes]
    return mode, axes


def get_available_ports():
    """
    返回 pyserial 枚举到的串口列表（ListPortInfo）。
    用 importlib 动态导入，避免某些 IDE 因解释器没配对而产生“无法解析 serial.tools”告警。
    """
    try:
        lp = importlib.import_module("serial.tools.list_ports")
    except Exception:
        return []
    try:
        return list(lp.comports())
    except Exception:
        return []

def print_available_ports(ports):
    if not ports:
        print("未检测到任何串口设备。请检查 USB/驱动/设备管理器。")
        return
    print("当前检测到的串口：")
    for p in ports:
        desc = (p.description or "").strip()
        hwid = (p.hwid or "").strip()
        extra = " | ".join([x for x in [desc, hwid] if x])
        if extra:
            print(f"  - {p.device}  ({extra})")
        else:
            print(f"  - {p.device}")


def open_serial_with_retry(port: str, baud: int, retries: int = 20, delay_s: float = 0.2):
    last_err = None
    for _ in range(max(1, retries)):
        try:
            # Windows 下串口通常是独占打开；PermissionError(13) 多数是被其它程序占用
            return serial.Serial(port, baud, timeout=0.0)  # 非阻塞
        except Exception as e:
            last_err = e
            time.sleep(delay_s)
    raise last_err


def main():
    ap = argparse.ArgumentParser(description="UR RTDE jog remote (serial -> speedJ)")
    ap.add_argument("--robot-ip", default=ROBOT_IP)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--failsafe-ms", type=int, default=FAILSAFE_MS)
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=0.2,
        help="整体速度缩放系数：最终速度 = JOG_QD * speed_scale（例如 0.3 更慢）",
    )
    ap.add_argument("--list-ports", action="store_true", help="仅列出当前可用串口并退出")
    args = ap.parse_args()

    ports = get_available_ports()
    print_available_ports(ports)
    if args.list_ports:
        return

    # 如果用户没显式改端口（仍用默认值），但默认端口当前不存在：
    # - 优先自动选择唯一的 CH340（或 VID:PID=1A86:7523）
    # - 否则在只有一个串口时自动选择它
    available_devices = {getattr(p, "device", None) for p in ports}
    if args.port not in available_devices:
        ch340 = [
            p
            for p in ports
            if ("CH340" in ((getattr(p, "description", "") or "").upper()))
            or ("1A86:7523" in ((getattr(p, "hwid", "") or "").upper()))
        ]
        chosen = None
        if len(ch340) == 1:
            chosen = ch340[0].device
        elif len(ports) == 1:
            chosen = ports[0].device

        if chosen:
            print(f"提示：未找到指定端口 {args.port}，自动改用 {chosen}。如需指定请使用 --port COMx。")
            args.port = chosen
        else:
            print(f"错误：未找到指定端口 {args.port}。请根据上面列表使用 --port COMx 指定。")
            sys.exit(2)

    print(f"尝试打开串口 {args.port} @ {args.baud} ...")
    try:
        ser = open_serial_with_retry(args.port, args.baud)
    except Exception as e:
        print(f"打开串口失败：{e!r}")
        print(
            "排查建议：\n"
            "- 关闭所有可能占用串口的程序：Arduino Serial Monitor / 串口调试助手 / 其它 Python 脚本 / 上位机软件\n"
            "- 在设备管理器确认设备端口号（插拔一次看 COM 号是否变化）\n"
            "- 尝试用管理员权限运行（少数情况下有用）\n"
            "- 换一根数据线/USB口，或重启电脑释放句柄\n"
            "- 如果你不确定端口：用本脚本打印的列表，改用 `--port COMx` 指定"
        )
        sys.exit(2)

    if args.speed_scale <= 0:
        print(f"错误：--speed-scale 必须 > 0（当前为 {args.speed_scale}）")
        sys.exit(2)
    if args.speed_scale != 1.0:
        print(f"速度缩放：speed_scale={args.speed_scale}（最终速度=JOG_QD*scale）")

    rtde = rtde_control.RTDEControlInterface(args.robot_ip, FREQ)

    buf = bytearray()
    last_rx_t = time.time()

    mode = MODE_DISABLE
    axes = [0] * 6
    prev_mode = MODE_DISABLE

    # 让循环更稳定：按固定节拍跑
    next_t = time.time()

    try:
        while True:
            # ---- 串口读入：拼接到缓冲 ----
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

                # 尽可能多切行，但最终只保留“最新的有效一行”
                latest = None
                while b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    s = line.decode(errors="ignore")
                    parsed = parse_line(s)
                    if parsed:
                        latest = parsed

                if latest:
                    mode, axes = latest
                    last_rx_t = time.time()

            # ---- failsafe：串口没更新就停 ----
            if (time.time() - last_rx_t) * 1000.0 > float(args.failsafe_ms):
                mode = MODE_DISABLE
                axes = [0] * 6

            # ---- 模式切换处理：从 ARM 切走时，立即 speedStop 一次 ----
            if prev_mode == MODE_ARM and mode != MODE_ARM:
                try:
                    rtde.speedStop()
                except Exception:
                    pass
            prev_mode = mode

            # ---- 发送 speedJ ----
            try:
                if mode == MODE_ARM:
                    qd = [axes[i] * JOG_QD[i] * float(args.speed_scale) for i in range(6)]
                    rtde.speedJ(qd, ACC_LIMIT, DT)
                else:
                    # 非机械臂模式：持续发 0（更平滑）
                    rtde.speedJ([0.0] * 6, ACC_LIMIT, DT)
            except Exception as e:
                # RTDE 异常：尝试重连（简单策略）
                print("RTDE error:", e)
                try:
                    rtde = rtde_control.RTDEControlInterface(ROBOT_IP, FREQ)
                except Exception as e2:
                    print("RTDE reconnect failed:", e2)

            # ---- 固定周期 sleep ----
            next_t += DT
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # 如果落后太多，重置节拍避免越跑越漂
                next_t = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        try:
            rtde.speedStop()
            rtde.stopScript()
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()
