"""Read-only action-feedback decoding, not a trajectory interpolator/validator."""
import math
from safety.joint_limits import JOINT_NAMES


def decode_feedback(message, goal_label, received):
    feedback = message.feedback
    names = list(feedback.joint_names)
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError("反馈关节顺序/数量无效")
    order = [names.index(name) for name in JOINT_NAMES]
    def positions(point):
        if len(point.positions) != 6: raise ValueError("反馈关节位置缺失")
        q = [float(point.positions[i]) for i in order]
        if not all(math.isfinite(v) for v in q): raise ValueError("反馈含NaN/Inf")
        return q
    duration = feedback.desired.time_from_start
    segment_end = duration.sec + duration.nanosec * 1e-9
    if not math.isfinite(segment_end) or segment_end < 0:
        raise ValueError("反馈时间字段无效")
    return dict(goal_label=goal_label, received=received,
        reference_q=positions(feedback.desired), actual_q=positions(feedback.actual),
        segment_end_time=segment_end,
        time_scope="JTC 2.54 reports NEXT NODE time here, not exact scaled clock")
