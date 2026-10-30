"""Faster simulation model entries keep the original hardware transitions."""
import pytest

from bxi_example_py_elf3.model_test_transitions import CxkEntryTransition
from test_model_test_manual import machine, PREFIX


@pytest.mark.parametrize("topic,duration", [("simulation/", .3), ("hardware/", 2.0)])
@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
def test_model_entry_duration_is_selected_from_actual_platform(monkeypatch, topic, duration, stage):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    ctx.ros_node.topic_prefix = topic
    tick(PREFIX + stage)
    assert sm.snapshot()["transition"]["duration"] == duration
    if topic == "simulation/":
        for _ in range(16):
            tick()
        assert sm.current_state_name == PREFIX + stage and not sm.in_transition
        assert states[stage].policy.steps > 0
        assert all((frame.kp > 0).all() for frame in frames)


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
def test_unknown_platform_does_not_enable_model(monkeypatch, stage):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    ctx.ros_node.topic_prefix = ""
    tick(PREFIX + stage)
    assert sm.snapshot()["transition"] is None


@pytest.mark.parametrize("duration", [0, -1, float("nan"), float("inf"), True])
def test_invalid_simulation_entry_duration_is_rejected(duration):
    with pytest.raises(ValueError, match="simulation_duration"):
        CxkEntryTransition.from_config("test", dict(duration=2., simulation_duration=duration))
