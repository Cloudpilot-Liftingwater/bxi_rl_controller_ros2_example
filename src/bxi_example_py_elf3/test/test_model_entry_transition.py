"""All three tests enter in 0.3 seconds on hardware and in simulation."""
import pytest

from bxi_example_py_elf3.model_test_transitions import CxkEntryTransition
from test_model_test_manual import machine, PREFIX


@pytest.mark.parametrize("topic", ["simulation/", "hardware/"])
@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
@pytest.mark.parametrize("initial", ["com.bxi.basic_actions/normal", "com.bxi.basic_actions/forward_back"])
def test_model_entry_finishes_in_point_three_seconds(monkeypatch, topic, stage, initial):
    sm, states, ctx, frames, tick = machine(monkeypatch, initial_state=initial)
    ctx.ros_node.topic_prefix = topic
    tick(PREFIX + stage)
    assert sm.snapshot()["transition"]["duration"] == .3
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
