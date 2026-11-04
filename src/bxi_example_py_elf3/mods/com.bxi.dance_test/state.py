"""One playback per entry. Completion and accuracy acceptance are separate."""
import time

import numpy as np

from bxi_example_py_elf3.control.elf3 import JOINT_NAMES, JOINT_POSITION_MIN, JOINT_POSITION_MAX
from bxi_example_py_elf3.framework.execution_observer import observe, log_message
from bxi_example_py_elf3.framework.mod_api import MotionReplayState
from bxi_example_py_elf3.model_test_safety import diagnostic

NORMAL = "com.bxi.basic_actions/normal"
ZERO = "com.bxi.basic_actions/zero_torque"
EXIT_BLEND = dict(type="running_blend", duration=1.0, curve="smootherstep",
                  sample_from=True, sample_to=True, advance_from=False, advance_to=True)
RETURN_NORMAL_BLEND = dict(type="running_blend", duration=0.1, curve="smootherstep",
                           sample_from=False, sample_to=True, advance_from=False, advance_to=True)
MIN_ACCURACY_SAMPLES = 50
ACCURACY_LIMITS_DEG = dict(mae=3.0, rmse=5.0, p95=10.0, maximum=20.0)


class DanceTestState(MotionReplayState):
    def __init__(self, name, state_id, policy, *, temperature_guard=None):
        super().__init__(name, state_id, policy, finish_state=NORMAL,
                         finish_trigger="dance_playback_complete", end_transition=EXIT_BLEND)
        self.result = "not_started"
        self._last_stamp = None
        self._last_feedback_at = 0.0
        self.temperature_guard = temperature_guard
        self._thermal_notice_sent = False
        self._accuracy_errors = []
        self._accuracy_expected = 0

    def _mark(self, status, reason="", **details):
        severity = "error" if status == "failed" else "warning" if status in ("cancelled", "interrupted") else "info"
        code = ' ['+details['code']+']' if details.get('code') else ''
        log_message(self.logger, severity, f"MODEL_TEST {self.name}{code} {status}: {reason}")
        observe("test_event", self.name, status, reason, severity=severity, **details)

    def on_prepare(self, ctx, from_state):
        self.result = "preparing"
        self._last_stamp = ctx.robot_joints.timestamp_ns
        self._last_feedback_at = time.monotonic()
        self._thermal_notice_sent = False
        super().on_prepare(ctx, from_state)

    def on_prepare_cancel(self, ctx, from_state):
        if self.result == "preparing":
            self.result = "cancelled"
            self._mark("cancelled", "entry cancelled before playback", code="ENTRY_CANCELLED", category="operator")

    def on_enter(self, ctx):
        super().on_enter(ctx)
        self.result = "running"
        self._last_stamp = ctx.robot_joints.timestamp_ns
        self._last_feedback_at = time.monotonic()
        self._accuracy_errors = []
        self._accuracy_expected = 0
        self._mark("started")
        if (self.policy.asset_info.get("source_type") != "smpl"
                and not self.policy.asset_info.get("motion_names_observed", False)):
            self.logger.warning("dance NPZ has no joint_names; order follows existing policy/model contract")

    def _fail(self, ctx, reason, *, code=None, category=None, details=None):
        if self.result == "failed":
            return
        self.result = "failed"
        inferred_code, inferred_category = diagnostic(reason)
        self._mark("failed", reason, code=code or inferred_code,
                   category=category or inferred_category, details=details or {})
        ctx.request_state(ZERO, trigger="dance_safety_fault", force=True)

    def _check_safety(self, ctx):
        now = time.monotonic()
        if ctx.robot_joints.timestamp_ns != self._last_stamp:
            self._last_stamp = ctx.robot_joints.timestamp_ns
            self._last_feedback_at = now
        if now - self._last_feedback_at > 0.2:
            raise ValueError("joint feedback stale for more than 0.2 s")
        measured = [ctx.robot_joints.position[ctx.robot_layout.index(n)] for n in JOINT_NAMES]
        if not np.isfinite(measured).all():
            raise ValueError("joint feedback contains non-finite angles")
        if ctx.is_orientation_unsafe(ctx.current_quat_xyzw):
            raise ValueError("unsafe orientation")
        if self.temperature_guard is not None:
            fault = self.temperature_guard.check(ctx)
            if fault is not None:
                self._fail(ctx, **fault)
                raise ValueError(fault["reason"])
            if (not self._thermal_notice_sent
                    and getattr(ctx.ros_node, "topic_prefix", "") == "simulation/"):
                self._thermal_notice_sent = True
                self._mark("not_assessed", "thermal: MuJoCo has no motor thermal model",
                           code="THERMAL_NOT_SIMULATED", category="temperature")

    def sample_running_frame(self, ctx, dt, *, advance):
        try:
            self._check_safety(ctx)
            frame = super().sample_running_frame(ctx, dt, advance=advance)
            return self._validate_frame(frame)
        except Exception as exc:
            # Preview calls also occur inside entry/exit transitions, outside
            # on_update. Never return an invalid frame to a blend/publisher.
            self._fail(ctx, str(exc))
            raise

    def _validate_frame(self, frame):
        names = frame.layout.names
        if len(names) != 29 or set(names) != set(JOINT_NAMES):
            raise ValueError("dance target lacks the complete named body29")
        indices = [JOINT_NAMES.index(n) for n in names]
        if not all(np.isfinite(a).all() for a in (frame.qpos, frame.kp, frame.kd)):
            raise ValueError("nonfinite dance target/gains")
        if np.any(frame.kp < 0) or np.any(frame.kd < 0):
            raise ValueError("negative dance gains")
        if np.any(frame.qpos < JOINT_POSITION_MIN[indices]) or np.any(frame.qpos > JOINT_POSITION_MAX[indices]):
            raise ValueError("dance target exceeds software joint limits")
        return frame

    def _record_accuracy(self, ctx, frame):
        self._accuracy_expected += 1
        model_by_name = {
            name: float(frame.qpos[index])
            for index, name in enumerate(frame.layout.names)
        }
        actual = np.asarray(
            [ctx.robot_joints.position[ctx.robot_layout.index(name)] for name in JOINT_NAMES],
            dtype=float,
        )
        if np.isfinite(actual).all():
            self._accuracy_errors.append(
                actual - np.asarray([model_by_name[name] for name in JOINT_NAMES], dtype=float)
            )

    def _report_accuracy(self):
        paired = len(self._accuracy_errors)
        if paired < MIN_ACCURACY_SAMPLES:
            log_message(
                self.logger, "warning",
                f"MODEL_TEST {self.name} ACCURACY_INSUFFICIENT: paired={paired} "
                f"required={MIN_ACCURACY_SAMPLES}",
            )
            observe("test_event", self.name, "accuracy_insufficient",
                    "not enough q_model/q_actual pairs", severity="warning",
                    code="TRACKING_DATA_INSUFFICIENT", category="tracking",
                    details={"paired_samples": paired, "required_samples": MIN_ACCURACY_SAMPLES})
            return
        errors_deg = np.degrees(np.asarray(self._accuracy_errors))
        absolute = np.abs(errors_deg)
        mae = np.mean(absolute, axis=0)
        rmse = np.sqrt(np.mean(errors_deg ** 2, axis=0))
        p95 = np.percentile(absolute, 95, axis=0)
        maximum = np.max(absolute, axis=0)
        bad = [
            f"{name}(mae={mae[i]:.1f},p95={p95[i]:.1f},max={maximum[i]:.1f})"
            for i, name in enumerate(JOINT_NAMES)
            if mae[i] > ACCURACY_LIMITS_DEG["mae"]
            or rmse[i] > ACCURACY_LIMITS_DEG["rmse"]
            or p95[i] > ACCURACY_LIMITS_DEG["p95"]
            or maximum[i] > ACCURACY_LIMITS_DEG["maximum"]
        ]
        status = "ACCURACY_FAIL" if bad else "ACCURACY_PASS"
        severity = "error" if bad else "info"
        summary = (
            f"MODEL_TEST {self.name} {status}: paired={paired} "
            f"mae={float(np.mean(mae)):.1f}deg p95={float(np.mean(p95)):.1f}deg "
            f"max={float(np.max(maximum)):.1f}deg"
        )
        if bad:
            summary += " bad=" + ",".join(bad)
        log_message(self.logger, severity, summary)
        observe("test_event", self.name, "accuracy_fail" if bad else "accuracy_pass",
                "q_actual minus q_model screening", severity=severity,
                code="TRACKING_ERROR" if bad else "TRACKING_OK", category="tracking",
                details={"paired_samples": paired, "bad_joints": bad,
                         "limits_deg": ACCURACY_LIMITS_DEG})

    def on_update(self, ctx, dt):
        if self.result != "running":
            return
        try:
            frame = self.sample_running_frame(ctx, dt, advance=True)
        except Exception as exc:
            self._fail(ctx, str(exc))
            return
        self._apply_frame(ctx, frame)
        self._record_accuracy(ctx, frame)
        if self.policy.finished():
            # A complete cursor says nothing about measured precision.
            self.result = "playback_complete"
            self._mark(self.result, "final motion frame consumed")
            self._report_accuracy()
            if ctx.request_state(NORMAL, trigger=self.finish_trigger, transition=EXIT_BLEND) is False:
                self._fail(ctx, "normal return state unavailable")

    def on_action(self, ctx, action_name):
        if action_name != "cancel":
            return False
        if self.result != "playback_complete":
            self.result = "cancelled"
            self._mark("cancelled", "operator requested normal", code="OPERATOR_RETURN_NORMAL", category="operator")
        if ctx.request_state(NORMAL, trigger="operator_return_normal", transition=RETURN_NORMAL_BLEND) is False:
            self._fail(ctx, "normal return state unavailable")
        return True

    def on_exit(self, ctx):
        if self.result == "running":
            self.result = "interrupted"
            self._mark("interrupted", "state changed before final motion frame")
