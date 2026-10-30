"""Native gamepad messages -> real event adapter/filters -> model state machine."""
import csv
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS

import pytest
import yaml

from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter
from test_execution_suite import module
from test_model_test_manual import machine, reach_motion_phase

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
NORMAL = "com.bxi.basic_actions/normal"
PREFIX = "com.bxi.execution_suite/"
CONTINUE = "com.bxi.suspended_tests/continue_suite"
EXIT = "com.bxi.suspended_tests/exit_test_mode"


def guard():
    return module("mods/com.bxi.execution_suite/remote_guard.py", "suite_guard_test").SuiteRemoteGuard()


def event_adapter():
    slots = {}
    for name in ("basic_actions", "suspended_tests", "dance_test", "execution_suite"):
        manifest = yaml.safe_load((ROOT / "mods" / ("com.bxi." + name) / "mod.yaml").read_text())
        slots.update({manifest["id"] + "/" + event: slot
                      for event, slot in manifest["events"].items()})
    return RemoteEventAdapter(slots)


def filtered_events(adapter, suspended, suite, msg, state):
    events = adapter.extract_events(msg)
    return suite(msg, suspended(msg, events, state), state)


@pytest.fixture(scope="module")
def native_messages(tmp_path_factory):
    binary = Path(os.environ.get("ELF3_GAMEPAD_TEST_BINARY", str(
        WORKSPACE / "build/remote_controller/remote_controller_execution_suite_gamepad_test")))
    if not binary.is_file():
        pytest.skip("build remote_controller with BUILD_TESTING=ON for native gamepad integration")
    trace = tmp_path_factory.mktemp("native-gamepad") / "messages.tsv"
    result = subprocess.run([str(binary), str(WORKSPACE / "src/remote_controller/config/xbox_default.yaml"),
                             str(trace)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    messages = {}
    for row in csv.reader(trace.read_text().splitlines(), delimiter="\t"):
        messages.setdefault(row[0], []).append((row[1], NS(**{
            f"btn_{slot}": int(value) for slot, value in enumerate(row[2:], 1)})))
    return messages


@pytest.mark.parametrize("action", ["continue", "dance", "sonic06", "sonic11"])
@pytest.mark.parametrize("release_order", range(6))
def test_native_chords_release_without_legacy_actions(native_messages, action, release_order):
    adapter = event_adapter()
    suite = guard()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_gamepad").TestRemoteGuard()
    expected = CONTINUE if action == "continue" else PREFIX + action
    state = "com.bxi.suspended_tests/sequence_running" if action == "continue" else NORMAL
    for phase, msg in native_messages[f"{action}_{release_order}"]:
        events = filtered_events(adapter, suspended, suite, msg, state)
        assert events == ([expected] if phase in ("request", "repeat") else []), (phase, events)
        # A handoff or completed clip may return to normal before keys are up.
        if phase == "held":
            state = NORMAL


@pytest.mark.parametrize("stage", ["dance", "sonic06", "sonic11"])
@pytest.mark.parametrize("when", ["entry", "active", "exit"])
def test_native_gamepad_can_enter_and_return_normal(native_messages, monkeypatch, stage, when):
    sm, states, ctx, frames, tick = machine(monkeypatch)
    adapter = event_adapter()
    suite = guard()
    suspended = module("mods/com.bxi.suspended_tests/remote_guard.py", "suspended_gamepad").TestRemoteGuard()

    def send(msg):
        events = filtered_events(adapter, suspended, suite, msg, sm.current_state_name)
        assert len(events) <= 1
        tick(events[0] if events else None)

    for phase, msg in native_messages[stage + "_0"]:
        if phase == "repeat":
            break
        send(msg)
    reach_motion_phase(sm, states, tick, stage, when)
    steps = states[stage].policy.steps
    for _, msg in native_messages["normal_rt0"]:
        send(msg)
    for _ in range(8):
        tick()
    assert sm.current_state_name == NORMAL and not sm.in_transition
    assert states[stage].policy.steps == steps
    assert states[stage].result == ("playback_complete" if when == "exit" else "cancelled")
    for _ in range(30):
        tick()
    assert sm.current_state_name == NORMAL


@pytest.mark.parametrize("escape,slot,value", [
    (NORMAL, 1, 1), (EXIT, 8, 2), ("com.bxi.basic_actions/zero_torque", 2, 1),
])
def test_escape_preempts_held_or_simultaneous_model(escape, slot, value):
    suite = guard()
    request = NS(btn_6=3, btn_8=3)
    assert suite(request, [PREFIX + "dance"], NORMAL) == [PREFIX + "dance"]
    msg = NS(btn_6=4, **{f"btn_{slot}": value})
    assert suite(msg, [PREFIX + "sonic06", escape], PREFIX + "dance") == [escape]
    assert suite(NS(btn_6=2, btn_8=3), ["com.bxi.dance_test/toggle"], NORMAL) == []
    suite(NS(), [], NORMAL)
    assert suite(request, [PREFIX + "dance"], NORMAL) == [PREFIX + "dance"]


def test_unavailable_raw_request_still_blocks_partial_release():
    suite = guard()
    assert suite(NS(btn_9=3, btn_8=3), [], NORMAL) == []
    assert suite(NS(btn_9=2, btn_8=3), ["com.bxi.suspended_tests/sequence"], NORMAL) == []
    suite(NS(), [], NORMAL)
    assert suite(NS(btn_9=2), ["com.bxi.suspended_tests/sequence"], NORMAL) == [
        "com.bxi.suspended_tests/sequence"]


def test_original_controls_are_unchanged_outside_the_suite():
    suite = guard()
    for event, buttons in [(NORMAL, {"btn_1": 1}),
                           ("com.bxi.basic_actions/pd_brake", {"btn_3": 1}),
                           ("com.bxi.suspended_tests/sequence", {"btn_9": 2})]:
        assert suite(NS(**buttons), [event], NORMAL) == [event]
