"""Rearm suite requests only after shoulder/face buttons return to neutral.

The same protocol is used by keyboard and gamepad. Inspect raw slots even
when an earlier Mod filter removed an unavailable request. Escape to normal
or zero torque always remains available; this filter never creates events.
"""

_NORMAL = "com.bxi.basic_actions/normal"
_ESCAPES = frozenset({
    _NORMAL,
    "com.bxi.basic_actions/zero_torque",
    "com.bxi.suspended_tests/exit_test_mode",
})
_SUSPENDED = "com.bxi.suspended_tests/"
_SUITE = "com.bxi.execution_suite/"
_MODELS = frozenset(_SUITE + stage for stage in ("dance", "sonic06", "sonic11"))
_TEST_ACTIONS = frozenset(_SUSPENDED + name for name in (
    "sequence", "running", "vibration", "whole_body_joint_test"))


def allowed_test_events(state):
    """Remote input allowlist; internal fault/IMU transitions remain untouched."""
    if state in ("com.bxi.basic_actions/pd_brake", "com.bxi.basic_actions/zero_torque"):
        allowed = {"com.bxi.basic_actions/pd_brake", _SUSPENDED + "test_mode"}
        if state.endswith("/pd_brake"):
            allowed.add(_NORMAL)
        return allowed | (_ESCAPES - {_NORMAL})
    if state == _NORMAL:
        return _MODELS | _ESCAPES
    if state in _MODELS or state == "com.bxi.dance_test/dance":
        return _ESCAPES
    if state == _SUITE + "ground_prepare":
        return _ESCAPES - {_NORMAL}
    if state == _SUSPENDED + "idle":
        return _TEST_ACTIONS | {_SUSPENDED + "exit_test_mode"}
    if state.startswith(_SUSPENDED):
        action = state.rsplit("/", 1)[-1]
        allowed = {_SUSPENDED + "exit_test_mode"}
        if action.startswith("sequence_"):
            allowed.add(_SUSPENDED + "sequence")
            if action == "sequence_running":
                allowed.add(_SUSPENDED + "continue_suite")
        elif _SUSPENDED + action in _TEST_ACTIONS:
            allowed.add(_SUSPENDED + action)
        return allowed
    return _ESCAPES - {_NORMAL}


class SuiteRemoteGuard:
    def __init__(self, *, isolate=False):
        self._awaiting_release = False
        self._isolate = isolate

    def __call__(self, message, events, current_state):
        if self._isolate:
            allowed = allowed_test_events(current_state)
            events = [event for event in events if event in allowed]
        neutral = all(
            int(getattr(message, f"btn_{slot}", 0)) == 0
            for slot in range(1, 11)
        )
        if neutral:
            self._awaiting_release = False
            return list(events)

        suite_request = (int(getattr(message, "btn_6", 0)) in (3, 4, 5)
                         or int(getattr(message, "btn_9", 0)) == 3)
        # Leave ordinary official controls alone outside suite requests/states.
        if not (self._isolate or self._awaiting_release or suite_request
                or current_state.startswith("com.bxi.execution_suite/")):
            return list(events)
        escapes = [event for event in events if event in _ESCAPES]
        if escapes:
            # Explicit zero torque wins over normal and any pending model.
            self._awaiting_release = True
            for stop in ("com.bxi.suspended_tests/exit_test_mode", "com.bxi.basic_actions/zero_torque"):
                if stop in escapes:
                    return [stop]
            return escapes
        if self._awaiting_release:
            return []

        if suite_request:
            self._awaiting_release = True
        return list(events)
