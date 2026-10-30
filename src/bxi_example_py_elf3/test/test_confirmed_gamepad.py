"""Immediate two-key profile, single-key stop and controller phase isolation."""
import csv
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS

import pytest
import yaml

from bxi_example_py_elf3.framework.mod_api import RobotControlState
from bxi_example_py_elf3.framework.runtime.state_machine import RobotStateMachine
from test_dance_execution import Logger, context
from test_execution_suite import module
from test_execution_suite_gamepad import event_adapter, filtered_events, ROOT, WORKSPACE
from test_model_test_manual import machine, setup_suite, reach_motion_phase

NORMAL = "com.bxi.basic_actions/normal"
SUITE = "com.bxi.execution_suite/"
SUSPENDED = "com.bxi.suspended_tests/"


def suite_guard():
    return module("mods/com.bxi.execution_suite/remote_guard.py", "isolated_guard").SuiteRemoteGuard(isolate=True)


@pytest.fixture(scope="module")
def confirmed_messages(tmp_path_factory):
    binary = WORKSPACE / "build/remote_controller/remote_controller_confirmed_gamepad_test"
    if not binary.is_file():
        pytest.skip("build remote_controller with BUILD_TESTING=ON for confirmed gamepad integration")
    trace = tmp_path_factory.mktemp("confirmed-gamepad") / "messages.tsv"
    result = subprocess.run([str(binary), str(WORKSPACE / "src/remote_controller/config/execution_suite_gamepad.yaml"),
                             str(trace)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    messages = {}
    for row in csv.reader(trace.read_text().splitlines(), delimiter="\t"):
        messages.setdefault(row[0], []).append((row[1], NS(**{
            f"btn_{slot}": int(value) for slot, value in enumerate(row[2:], 1)})))
    return messages


CASES = [(action, press, release)
         for action in ("pd", "idle", "continue", "dance", "sonic06", "sonic11")
         for press in range(2)
         for release in range(2)]


@pytest.mark.parametrize("action,press,release", CASES)
def test_all_native_press_and_release_orders(confirmed_messages, action, press, release):
    states = {"pd": "com.bxi.basic_actions/zero_torque", "idle": "com.bxi.basic_actions/pd_brake",
              "continue": SUSPENDED + "sequence_running"}
    expected = {"pd": "com.bxi.basic_actions/pd_brake", "idle": SUSPENDED + "test_mode",
                "continue": SUSPENDED + "continue_suite", "zero": SUSPENDED + "exit_test_mode"}
    state = states.get(action, NORMAL)
    wanted = expected.get(action, SUITE + action)
    adapter = event_adapter()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_confirmed").TestRemoteGuard()
    suite = suite_guard()
    for phase, msg in confirmed_messages[f"{action}_p{press}_r{release}"]:
        events = filtered_events(adapter, suspended, suite, msg, state)
        assert events == ([wanted] if phase in ("request", "repeat") else []), (phase, events)


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
@pytest.mark.parametrize("when", ["entry", "active", "exit"])
def test_confirmed_profile_normal_works_with_initiating_modifiers_held(
        confirmed_messages, monkeypatch, stage, when):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    adapter = event_adapter()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_confirmed").TestRemoteGuard()
    suite = suite_guard()

    def send(msg):
        events = filtered_events(adapter, suspended, suite, msg, sm.current_state_name)
        assert len(events) <= 1
        tick(events[0] if events else None)

    for phase, msg in confirmed_messages[stage + "_p0_r0"]:
        if phase == "repeat":
            break
        send(msg)
    reach_motion_phase(sm, states, tick, stage, when)
    steps = states[stage].policy.steps
    for _, msg in confirmed_messages["escape"]:
        send(msg)
    for _ in range(8):
        tick()
    assert sm.current_state_name == NORMAL and not sm.in_transition
    assert states[stage].policy.steps == steps
    assert states[stage].result == ("playback_complete" if when == "exit" else "cancelled")


@pytest.mark.parametrize("state", [NORMAL, SUITE + "dance", SUITE + "sonic06", SUITE + "sonic11",
                                   SUSPENDED + "idle", SUSPENDED + "sequence_running"])
@pytest.mark.parametrize("buttons", [dict(btn_10=8), dict(btn_6=1), dict(btn_6=2),
                                     dict(btn_5=1), dict(btn_4=1)])
def test_other_remote_entries_cannot_bypass_isolation(state, buttons):
    adapter = event_adapter()
    msg = NS(**buttons)
    events = adapter.extract_events(msg)
    assert events  # This is an actual legacy event, not an empty test message.
    assert suite_guard()(msg, events, state) == []


@pytest.mark.parametrize("state", [NORMAL, SUITE + "dance", SUITE + "sonic06", SUITE + "sonic11"])
def test_pd_brake_cannot_interrupt_ground_model_testing(state):
    pd = "com.bxi.basic_actions/pd_brake"
    assert suite_guard()(NS(btn_3=1), [pd], state) == []


def test_continue_is_only_accepted_during_sequence_running():
    event = SUSPENDED + "continue_suite"
    for state in (NORMAL, SUSPENDED + "idle", SUSPENDED + "sequence_joint", SUSPENDED + "sequence_vibration"):
        assert suite_guard()(NS(btn_9=3), [event], state) == []
    assert suite_guard()(NS(btn_9=3), [event], SUSPENDED + "sequence_running") == [event]


def test_old_keyboard_original_test_actions_remain_available_in_test_idle():
    for action, slot in (("whole_body_joint_test", 7), ("vibration", 10), ("running", 9)):
        event = SUSPENDED + action
        assert suite_guard()(NS(**{f"btn_{slot}": 1}), [event], SUSPENDED + "idle") == [event]


def test_isolation_does_not_clear_imu_protection():
    assert suite_guard()(NS(btn_1=1), [NORMAL], "com.bxi.basic_actions/imu_protection") == []


def test_plugin_enables_isolation_only_with_the_suite(monkeypatch):
    _, definition, *_ = setup_suite(monkeypatch)
    assert definition.remote_event_filter._isolate
    monkeypatch.delenv("BXI_EXECUTION_SUITE")
    # The guard default keeps the original, non-suite input behavior.
    guard = module("mods/com.bxi.execution_suite/remote_guard.py", "plain_guard").SuiteRemoteGuard()
    event = "com.bxi.basic_actions/lie_down"
    assert guard(NS(btn_10=8), [event], NORMAL) == [event]


@pytest.mark.parametrize("hold_ms", [-1, 10001])
def test_parser_rejects_invalid_confirmation_times(tmp_path, confirmed_messages, hold_ms):
    profile = yaml.safe_load((WORKSPACE / "src/remote_controller/config/execution_suite_gamepad.yaml").read_text())
    profile["outputs"]["gestures"]["bindings"][0]["hold_ms"] = hold_ms
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(profile))
    result = subprocess.run([str(WORKSPACE / "build/remote_controller/remote_controller_confirmed_gamepad_test"),
                             str(path)], capture_output=True, text=True, timeout=15)
    assert result.returncode != 0 and "hold_ms" in result.stderr


@pytest.mark.parametrize("state", [NORMAL, SUITE + "dance", SUITE + "sonic06", SUITE + "sonic11",
                                   SUITE + "ground_prepare", SUSPENDED + "sequence_running",
                                   SUSPENDED + "sequence_vibration", SUSPENDED + "idle"])
def test_single_key_stop_wins_over_normal_and_interlocks(confirmed_messages, state):
    adapter = event_adapter()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_stop").TestRemoteGuard()
    suite = suite_guard()
    # The handoff's existing full-release barrier must not delay an explicit stop.
    suspended.observe_state(SUSPENDED + "sequence_running")
    suspended.observe_state(state)
    suite(NS(btn_6=3), [SUITE + "dance"], NORMAL)
    msg = dict(confirmed_messages["stop"])["request"]
    events = filtered_events(adapter, suspended, suite, msg, state)
    assert events == [SUSPENDED + "exit_test_mode"]


def test_stop_priority_and_release_cannot_reenable_normal():
    suite = suite_guard()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_stop").TestRemoteGuard()
    stop = SUSPENDED + "exit_test_mode"
    assert suite(NS(btn_8=2, btn_1=1), [NORMAL, stop], NORMAL) == [stop]
    assert suspended(NS(btn_8=2), [stop], NORMAL) == [stop]
    assert suspended(NS(btn_8=3, btn_1=1), [NORMAL], "com.bxi.basic_actions/pd_brake") == []
    suspended(NS(), [], "com.bxi.basic_actions/pd_brake")
    assert suspended(NS(btn_1=1), [NORMAL], "com.bxi.basic_actions/pd_brake") == [NORMAL]


@pytest.mark.parametrize("state", ["com.bxi.basic_actions/pd_brake", NORMAL, SUITE + "ground_prepare",
    SUITE + "dance", SUITE + "sonic06", SUITE + "sonic11", SUSPENDED + "idle",
    SUSPENDED + "sequence_joint", SUSPENDED + "sequence_vibration", SUSPENDED + "sequence_running",
    SUSPENDED + "running", SUSPENDED + "vibration", SUSPENDED + "whole_body_joint_test"])
def test_stop_message_reaches_zero_through_actual_routes(confirmed_messages, state):
    # Exercise the real route definitions and state machine, not just an input
    # slot/filter assertion. Dynamics are unnecessary for an instant state exit.
    zero = "com.bxi.basic_actions/zero_torque"
    exit_event = SUSPENDED + "exit_test_mode"
    routes = {}
    for name in ("suspended_tests", "execution_suite"):
        manifest = yaml.safe_load((ROOT / "mods" / ("com.bxi." + name) / "mod.yaml").read_text())
        for route in manifest["routes"]:
            event = route["event"] if "/" in route["event"] else manifest["id"] + "/" + route["event"]
            if event != exit_event:
                continue
            src = route["from"] if "/" in route["from"] else manifest["id"] + "/" + route["from"]
            dst = route["to"] if "/" in route["to"] else manifest["id"] + "/" + route["to"]
            routes[src] = dst
    assert routes[state] == zero
    class Hold(RobotControlState):
        def on_update(self, ctx, dt):
            pass
    factories = {name: Hold(name, i + 1) for i, name in enumerate((state, zero))}
    for value in factories.values():
        value._bind_logger(Logger())
    config = dict(initial_state=state, graph={"validate": False}, default_transition="instant",
        transition_profiles={"instant": {"type": "instant"}}, states={
            state: {"transitions": {"on_event": {exit_event: {"to": routes[state], "transition": "instant"}}}}, zero: {}})
    ctx, *_ = context()
    sm = RobotStateMachine(ctx, config, factories, logger=Logger())
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "route_stop").TestRemoteGuard()
    suspended.observe_state(SUSPENDED + "sequence_running")
    suspended.observe_state(state)
    msg = dict(confirmed_messages["stop"])["request"]
    events = filtered_events(event_adapter(), suspended, suite_guard(), msg, state)
    sm.update(.02, events)
    assert sm.current_state_name == zero and not sm.in_transition


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
@pytest.mark.parametrize("phase", ["entry", "active", "exit", "resource_preparation"])
def test_view_preempts_model_phases_and_publishes_actual_zero_frame(
        monkeypatch, confirmed_messages, stage, phase):
    import numpy as np
    sm, states, ctx, frames, tick = machine(monkeypatch, include_zero=True)
    if phase == "resource_preparation":
        states[stage].required_resources[0].status = "loading"
    tick(SUITE + stage)
    if phase == "resource_preparation":
        assert sm.snapshot()["preparing"] is not None
    else:
        reach_motion_phase(sm, states, tick, stage, phase)
    steps = states[stage].policy.steps
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "phase_stop").TestRemoteGuard()
    # Retain both release barriers to prove View bypasses them while a chord
    # or pending model request is still present.
    suspended.observe_state(SUSPENDED + "sequence_running")
    suspended.observe_state(sm.current_state_name)
    suite = suite_guard()
    suite(NS(btn_6=3), [SUITE + "dance"], NORMAL)
    msg = dict(confirmed_messages["stop"])["request"]
    events = filtered_events(event_adapter(), suspended, suite, msg, sm.current_state_name)
    assert events == [SUSPENDED + "exit_test_mode"]
    tick(events[0])
    assert sm.current_state_name == "com.bxi.basic_actions/zero_torque"
    assert not sm.in_transition and sm.snapshot()["preparing"] is None
    # Finish any pending resource load after the stop; it must never restart
    # the cancelled model. Use the production ZeroTorqueState for publication.
    states[stage].required_resources[0].status = "ready"
    for _ in range(20):
        tick()
        assert sm.current_state_name == "com.bxi.basic_actions/zero_torque"
        for field in ("kp", "kd", "vel", "torque"):
            np.testing.assert_array_equal(getattr(frames[-1], field), np.zeros(29))
    assert states[stage].policy.steps == steps


def test_startup_sync_does_not_execute_remote_state_requests():
    adapter = event_adapter()
    stop = NS(btn_8=2)
    # BxiExample.joy_callback explicitly selects sync_only until reset step2.
    assert adapter.extract_events(stop, sync_only=True) == []
    assert adapter.extract_events(stop) == []  # An old held key is not a new edge.
    adapter.extract_events(NS())
    assert adapter.extract_events(stop) == [SUSPENDED + "exit_test_mode"]
