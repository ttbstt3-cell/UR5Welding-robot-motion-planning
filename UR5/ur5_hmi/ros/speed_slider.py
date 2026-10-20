"""Non-blocking wrapper of the installed UR driver's speed-slider service.

No startup command, resend, set_tcp, motion goal, controller switch or URScript.
"""
import math
import time
from ur_msgs.srv import SetSpeedSliderFraction

SERVICE = "/io_and_status_controller/set_speed_slider"


def speed_request_reasons(state, now=None):
    now = time.monotonic() if now is None else now
    reasons = []
    if not state.speed_slider_service_ready:
        reasons.append("示教器调速服务不可用")
    if (len(state.positions) != 6 or not all(math.isfinite(q) for q in state.positions)
            or now - state.joint_stamp > 1 or state.robot_link_alive is not True):
        reasons.append("没有新鲜真机关节反馈/连接")
    if state.robot_mode != 7 or state.program_running is not True:
        reasons.append("机器人需RUNNING且External Control运行")
    if state.safety_mode != 1 or not state.controller_active:
        reasons.append("需NORMAL安全状态及active的scaled控制器")
    if (state.speed_scaling is None or not math.isfinite(state.speed_scaling)
            or not 0 <= state.speed_scaling <= 100 or now - state.speed_scaling_stamp > 1):
        reasons.append("实际倍率反馈无效或过期")
    return reasons


class SpeedSliderClient:
    def __init__(self, node, get_state, on_done, clock=time.monotonic):
        self.client = node.create_client(SetSpeedSliderFraction, SERVICE)
        self.get_state, self.on_done, self.clock = get_state, on_done, clock
        self.future = None
        self.target = None
        self.started = 0.
        self.timed_out = False
        self.timer = node.create_timer(.2, self._check_timeout)

    @property
    def busy(self): return self.future is not None

    def ready(self): return self.client.service_is_ready()

    def request(self, percent):
        if self.busy:
            return False  # Never overlap or reorder requests.
        try: percent = float(percent)
        except (TypeError, ValueError): percent = float("nan")
        if not math.isfinite(percent) or not 1 <= percent <= 100:
            self.on_done(False, 0., "倍率必须为1%～100%；不支持超过已验证轨迹的100%")
            return False
        state = self.get_state()
        reasons = speed_request_reasons(state, self.clock())
        if not self.ready(): reasons.append("调速服务连接不可用")
        if reasons:
            self.on_done(False, percent, "；".join(reasons))
            return False
        request = SetSpeedSliderFraction.Request()
        request.speed_slider_fraction = percent / 100.
        self.target, self.started, self.timed_out = percent, self.clock(), False
        try:
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self._done)
        except Exception as exc:
            self.future = None
            self.on_done(False, percent, "调速请求异常：" + str(exc))
            return False
        return True

    def _check_timeout(self):
        if self.busy and not self.timed_out and self.clock() - self.started > 3.:
            self.timed_out = True
            # The request may still be applied: don't cancel/retry and don't
            # permit a newer command to race an unresolved older request.
            self.on_done(False, self.target, "调速请求超时，结果UNKNOWN；请核对示教器，等待原请求返回")

    def _done(self, future):
        if future is not self.future: return
        late, target = self.timed_out, self.target
        self.future = None
        try:
            ok = bool(future.result().success)
            message = ("控制器已接受倍率请求；实际倍率以实时反馈为准" if ok else "控制器拒绝倍率请求")
        except Exception as exc:
            ok, message = False, "调速服务返回异常，实际状态UNKNOWN：" + str(exc)
        self.on_done(ok, target, ("延迟返回：" if late else "") + message)
