"""Real Humble logger regressions: progress -> operator return -> fault."""
import uuid

import pytest
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity

from test_execution_suite import module
from test_model_test_manual import machine, reach_motion_phase, NORMAL, PREFIX


def ros_logger():
    logger = get_logger("model_test_regression_" + uuid.uuid4().hex)
    logger.set_level(LoggingSeverity.DEBUG)
    return logger


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
def test_real_ros_model_progress_cancel_and_fault_share_no_severity_callsite(monkeypatch, stage):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    state = states[stage]
    state._bind_logger(ros_logger())
    for status in ("started", "cancelled", "not_assessed", "failed", "interrupted", "playback_complete"):
        state._mark(status, "regression")
    assert {value["severity"] for value in state.logger.contexts.values()} >= {
        LoggingSeverity.INFO, LoggingSeverity.WARN, LoggingSeverity.ERROR}


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
def test_real_ros_logger_does_not_crash_operator_return(monkeypatch, stage):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    states[stage]._bind_logger(ros_logger())
    tick(PREFIX + stage)
    reach_motion_phase(sm, states, tick, stage, "active")
    tick(NORMAL)
    for _ in range(8):
        tick()
    assert sm.current_state_name == NORMAL and not sm.in_transition


def test_broken_log_sink_cannot_prevent_return_or_drop_structured_cancel(monkeypatch):
    from bxi_example_py_elf3.framework import execution_observer
    events = []
    class BrokenLogger:
        def broken(self, message):
            raise OSError("synthetic log sink unavailable")
        info = warning = error = broken
    sm, states, ctx, frames, tick = machine(monkeypatch)
    states["dance"]._bind_logger(BrokenLogger())
    class Observer:
        def test_event(self, *args, **kwargs):
            events.append((args, kwargs))
    monkeypatch.setattr(execution_observer, "_host", Observer())
    tick(PREFIX + "dance")
    reach_motion_phase(sm, states, tick, "dance", "active")
    tick(NORMAL)
    for _ in range(8):
        tick()
    assert sm.current_state_name == NORMAL
    assert any(args[1] == "cancelled" for args, _ in events)
