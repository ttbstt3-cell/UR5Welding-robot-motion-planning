import time
from typing import List, Optional

import rtde_control
import rtde_receive

# ------------ 可调参数 ------------
ROBOT_IP = "192.168.0.10"
FREQUENCY = 125.0  # 可改回 500.0 试验
DT = 1.0 / FREQUENCY
RUN_MOTION = True  # 先只测连接，确认不崩溃后再改 True
# 优先使用 ExternalControl URCap，避免默认脚本触发崩溃
PREFERRED_USE_EXT = False  # Windows 不支持 ExternalControl URCap，禁用
CONTROL_PORT = 50002  # ExternalControl 默认端口（已禁用，仅留作参考）
# ---------------------------------

# 兼容获取 flag，避免属性不存在时报错
try:
    FLAG_USE_EXT_UR_CAP = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
except Exception:
    FLAG_USE_EXT_UR_CAP = 0
try:
    FLAG_NO_WAIT = rtde_control.RTDEControlInterface.FLAG_NO_WAIT
except Exception:
    FLAG_NO_WAIT = 0


def create_control_interface(ip: str) -> rtde_control.RTDEControlInterface:
    """分多种参数组合尝试创建 RTDEControlInterface，避免直接崩溃。"""
    candidates = []

    # 回退方案
    # Windows 环境仅尝试标准控制脚本
    candidates.append(("默认无flag", (ip,), {}))
    candidates.append(("带频率", (ip, FREQUENCY), {}))

    last_err: Optional[Exception] = None
    for desc, args, kwargs in candidates:
        try:
            print(f"尝试 {desc}: args={args}, kwargs={kwargs}")
            ctrl = rtde_control.RTDEControlInterface(*args, **kwargs)
            print(f"✓ RTDEControlInterface 连接成功（{desc}）")
            return ctrl
        except Exception as e:
            print(f"⚠️ {desc} 失败: {e}")
            last_err = e
            time.sleep(0.1)

    raise RuntimeError(f"所有 RTDEControlInterface 连接方式均失败: {last_err}")


def main() -> None:
    # 访问冲突(0xC0000005)属于原生扩展崩溃；启用 faulthandler 可尽量打印栈。
    try:
        import faulthandler

        faulthandler.enable()
    except Exception:
        pass

    print(f"正在连接机器人 {ROBOT_IP}，频率: {FREQUENCY}Hz")

    # 先建 RTDEReceive，验证网络/端口可用
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("✓ RTDEReceiveInterface 连接成功")

    # 再尝试建立 RTDEControl，按多套参数尝试
    rtde_c = create_control_interface(ROBOT_IP)

    if not RUN_MOTION:
        print("仅连接测试完成（未下发运动指令）。如需运动，将 RUN_MOTION 设为 True。")
        try:
            rtde_c.stopScript()
        except Exception:
            pass
        return

    # 运动参数（单位：rad/s 与 rad/s^2）
    ACC = 1.0
    V = 0.3
    JOINT_SPEED: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, V]  # 只让第6轴转动

    duration_s = 2.0
    n_cycles = int(duration_s * FREQUENCY)

    try:
        for i in range(n_cycles):
            t_start = rtde_c.initPeriod()
            rtde_c.speedJ(JOINT_SPEED, ACC, DT)

            # 打印会影响实时性，这里降频打印
            if i % 50 == 0:
                print(rtde_r.getActualQ())

            rtde_c.waitPeriod(t_start)
    finally:
        # 无论发生什么都尽量让机器人停下来
        try:
            rtde_c.speedStop()
        except Exception:
            pass
        try:
            rtde_c.stopScript()
        except Exception:
            pass

    print("done")


if __name__ == "__main__":
    main()
