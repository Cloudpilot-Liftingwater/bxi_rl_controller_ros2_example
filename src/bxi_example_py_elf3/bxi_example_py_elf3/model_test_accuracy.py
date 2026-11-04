"""Bounded, causal control-rate screening; full-rate analysis stays in elf3-eval."""
from collections import deque
from threading import Thread

import numpy as np

from .control.elf3 import JOINT_NAMES
from .framework.execution_observer import log_message, observe


class TrackingRun:
    # Engineering screening approved for the first baseline collection.
    limits = {"mae": 3.0, "rmse": 5.0, "p95": 10.0, "maximum": 20.0}
    minimum_span = 2.0
    minimum_inferences = 50
    minimum_coverage = .95
    max_samples = 20000

    def __init__(self, state, period, logger):
        self.state, self.logger = state, logger
        self.period = float(period)
        self.age_ns = int(5 * self.period * 1e9)
        self.pending = None
        self.commands = deque(maxlen=64)
        self.errors = np.empty((self.max_samples, len(JOINT_NAMES)))
        self.samples = 0
        self.inferences = set()
        self.sequence = 0
        self.first_stamp = self.last_stamp = None
        self.seen_stamp = None
        self.covered_s = 0.0
        self.issues = set()
        self.observer_failed = False
        self.fault = None
        self.worker = None
        self.report = None

    def stage(self, frame, *, blend=False):
        self.pending = (np.array([frame.qpos[frame.layout.index(name)]
                                  for name in JOINT_NAMES]), bool(blend))

    def command_published(self, msg, state, transition_active=False):
        pending, self.pending = self.pending, None
        if state != self.state or pending is None or transition_active or pending[1]:
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.commands and stamp <= self.commands[-1][0]:
            self.issues.add("command_clock_discontinuity")
            self.commands.clear()
        self.sequence += 1
        self.commands.append((stamp, self.sequence, pending[0]))
        if self.first_stamp is None:
            self.first_stamp = stamp

    def sample(self, joints):
        # This is called BEFORE the next inference. Both timestamps are local
        # ROS receipt/publication clock stamps, not a physical motor latency.
        stamp = int(joints.timestamp_ns)
        if self.seen_stamp is not None:
            if stamp == self.seen_stamp:
                return
            if stamp < self.seen_stamp:
                self.issues.add("feedback_clock_discontinuity")
                self.commands.clear()
                self.seen_stamp = stamp
                return
        self.seen_stamp = stamp
        command = next((c for c in reversed(self.commands) if c[0] <= stamp), None)
        if command is None:
            return
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        if stamp - command[0] > self.age_ns:
            return
        actual = np.asarray([joints.position[joints.layout.index(n)] for n in JOINT_NAMES])
        if not np.isfinite(actual).all():
            self.issues.add("nonfinite_feedback")
            return
        if self.samples >= self.max_samples:
            self.issues.add("sample_capacity_exceeded")
            return
        error = np.degrees(actual - command[2])
        self.errors[self.samples] = error
        self.samples += 1
        self.inferences.add(command[1])
        # Coverage uses elapsed receipt-clock time, capping a sample's support
        # at one control period so recorder gaps cannot become valid coverage.
        previous = getattr(self, "_paired_stamp", stamp)
        self.covered_s += min(max((stamp - previous) / 1e9, 0.0), self.period)
        self._paired_stamp = stamp
        bad = np.flatnonzero(np.abs(error) > self.limits["maximum"])
        if bad.size:
            i = int(bad[np.argmax(np.abs(error[bad]))])
            self.fault = {"joint": JOINT_NAMES[i], "observed_deg": float(error[i]),
                          "limit_deg": self.limits["maximum"]}

    def finish(self, completed):
        # No sorting, percentile calculation, or file I/O in the control cycle.
        if self.worker is None:
            self.worker = Thread(target=self._summarize, args=(completed,),
                                 name="model-test-summary", daemon=True)
            self.worker.start()

    def _summarize(self, completed):
        span = (self.last_stamp - self.first_stamp) / 1e9 if (
            self.first_stamp is not None and self.last_stamp is not None) else 0.0
        coverage = min(1.0, self.covered_s / span) if span > 0 else 0.0
        reasons = sorted(self.issues)
        if self.observer_failed:
            reasons.append("observer_failed")
        if span < self.minimum_span:
            reasons.append("duration_below_2s")
        if len(self.inferences) < self.minimum_inferences:
            reasons.append("independent_targets_below_50")
        if coverage < self.minimum_coverage:
            reasons.append("time_coverage_below_95pct")
        rows, bad = {}, []
        if self.samples:
            e = self.errors[:self.samples]
            for i, name in enumerate(JOINT_NAMES):
                a = np.abs(e[:, i])
                row = {"mae": float(a.mean()), "rmse": float(np.sqrt(np.mean(e[:, i]**2))),
                       "p95": float(np.percentile(a, 95)), "maximum": float(a.max()),
                       "bias": float(e[:, i].mean())}
                rows[name] = row
                violations = [f"{k}={row[k]:.2f}>{v:g}" for k, v in self.limits.items() if row[k] > v]
                if violations:
                    bad.append((name, violations))
        status = "ABORTED" if not completed else "INSUFFICIENT_DATA" if reasons else "FAIL" if bad else "PASS"
        self.report = dict(status=status, paired=self.samples, independent_targets=len(self.inferences),
                           span_s=span, coverage=coverage, reasons=reasons, joints=rows,
                           alignment="causal_publication_vs_feedback_receipt_clock",
                           sampling="control_rate", limits_deg=self.limits)
        severity = "error" if status == "FAIL" else "info" if status == "PASS" else "warning"
        suffix = (" reason=" + ",".join(reasons)) if reasons else ""
        if rows:
            suffix += " worst_joint_deg=" + ",".join(
                f"{k}:{max(r[k] for r in rows.values()):.2f}" for k in self.limits)
        # Always list every failing joint; one summary and one line per joint.
        log_message(self.logger, severity, f"MODEL_TEST {self.state} {status} "
                    f"paired={self.samples} targets={len(self.inferences)} coverage={coverage:.1%}{suffix}")
        for name, violations in bad:
            log_message(self.logger, "error", f"MODEL_TEST {self.state} joint={name} " + " ".join(violations))
        observe("test_event", self.state, "accuracy_" + status.lower(),
                "control-rate q_actual-q_model screening", severity=severity,
                category="tracking", details=self.report)
