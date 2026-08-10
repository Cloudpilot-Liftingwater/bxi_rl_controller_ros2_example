import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import zmq

import bxi_example_py_elf3.inference.sonic as sonic_module
from bxi_example_py_elf3.inference.sonic import (
    MODEL_INPUT_DIM,
    NUM_JOINTS,
    WINDOW,
    SmplReferenceFrame,
    SonicTeleopPolicy,
)


class _FakeSession:
    def __init__(self):
        self.calls = 0

    def run(self, _output_names, _inputs):
        self.calls += 1
        return [np.ones((1, NUM_JOINTS), dtype=np.float32)]


class _FakeOutboundQueue:
    def __init__(self, *, fail_put=False):
        self.fail_put = fail_put
        self.messages = []

    def put_nowait(self, message):
        if self.fail_put:
            raise queue.Full
        self.messages.append(message)


def _decode_control(policy, outbound, index=-1):
    message = outbound.messages[index]
    return sonic_module._decode_packed_message(message, policy.smpl_control_zmq_topic)


def _accept_live(policy, frame):
    if not policy._reference_allowed_after_reset(frame):
        return False
    policy.latest_live_ref = frame
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    return True


def _make_policy(monkeypatch, *, require_live_reference=True):
    def fake_load_reference(self):
        self.ref_term1 = np.zeros((WINDOW, 72), dtype=np.float32)
        self.ref_root_quat = np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (WINDOW, 1),
        )
        self.ref_wrist = np.zeros((WINDOW, 6), dtype=np.float32)
        self.ref_anchor_quat = self.ref_root_quat.copy()

    def fake_init_onnx(self):
        self.session = _FakeSession()
        self.input_info = SimpleNamespace(name="obs_dict", shape=(1, MODEL_INPUT_DIM))
        self.output_info = SimpleNamespace(name="action", shape=(1, NUM_JOINTS))
        self.input_buffer = np.zeros((1, MODEL_INPUT_DIM), dtype=np.float32)

    def fake_init_zmq(self):
        self.zmq_context = None
        self.zmq_socket = None
        self.zmq_control_socket = None
        self.zmq_poller = None
        self._zmq_inbound_queue = queue.Queue(maxsize=1)
        self._zmq_outbound_queue = _FakeOutboundQueue()
        self._zmq_stop_event = threading.Event()
        self._zmq_ready_event = threading.Event()
        self._zmq_ready_event.set()
        self._zmq_thread = None
        self._zmq_io_thread_id = None
        self._zmq_control_available = True
        self._zmq_start_error = None

    monkeypatch.setattr(SonicTeleopPolicy, "_load_stream_reference", fake_load_reference)
    monkeypatch.setattr(SonicTeleopPolicy, "_init_onnx", fake_init_onnx)
    monkeypatch.setattr(SonicTeleopPolicy, "_init_zmq", fake_init_zmq)
    policy = SonicTeleopPolicy(
        model_onnx_path="unused.onnx",
        stream_reference_npz="unused.npz",
        use_smpl_ref_zmq=False,
        require_live_reference=require_live_reference,
        yaw_bias_rad=0.0,
    )
    policy.source_blend_duration_s = 0.0
    return policy


def _frame(
    *,
    epoch=1,
    source_stale=False,
    source_age_ms=0.0,
    sequence=1,
    playout_seq=1,
    consumer_session=None,
):
    identity = np.tile(
        np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
        (WINDOW, 1),
    )
    return SmplReferenceFrame(
        term1_local=np.zeros((WINDOW, 72), dtype=np.float32),
        root_quat=identity.copy(),
        wrist=np.zeros((WINDOW, 6), dtype=np.float32),
        anchor_quat=identity.copy(),
        frame_index=100,
        sequence=sequence,
        stream_epoch=epoch,
        source_stale=source_stale,
        source_age_ms=source_age_ms,
        newest_frame_index=109,
        lead_frames=9,
        valid_horizon=10,
        clamp_slots=0,
        playout_seq=playout_seq,
        consumer_session=consumer_session,
    )


def _robot_observation():
    return (
        np.zeros(NUM_JOINTS, dtype=np.float32),
        np.zeros(NUM_JOINTS, dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float32),
    )


def _live_fields(frame_count=WINDOW):
    return {
        "source_ready": np.array([True], dtype=bool),
        "source_stream_mode": np.array([1], dtype=np.int32),
        "source_calibration_ready": np.array([True], dtype=bool),
        "term1_local": np.zeros((frame_count, 72), dtype=np.float32),
        "root_quat": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (frame_count, 1),
        ),
        "wrist": np.zeros((frame_count, 6), dtype=np.float32),
        "frame_index": np.array([41], dtype=np.int64),
        "newest_frame_index": np.array([50], dtype=np.int64),
        "stream_epoch": np.array([7], dtype=np.int64),
        "valid_horizon": np.array([WINDOW], dtype=np.int64),
        "clamp_slots": np.array([0], dtype=np.int64),
        "playout_seq": np.array([123], dtype=np.int64),
        "consumer_session": np.array([456], dtype=np.int64),
    }


def test_frame_parses_bridge_epoch_stale_and_cursor_metadata(monkeypatch):
    policy = _make_policy(monkeypatch)
    fields = _live_fields()
    fields.update(
        {
            "source_stale": np.array([1], dtype=np.uint8),
            "source_age_ms": np.array([612.5], dtype=np.float32),
            "playback_hold": np.array([1], dtype=np.uint8),
            "lead_frames": np.array([9], dtype=np.int64),
        }
    )

    frame = policy._frame_from_fields(fields)

    assert frame.frame_index == 41
    assert frame.newest_frame_index == 50
    assert frame.stream_epoch == 7
    assert frame.source_stale is True
    assert frame.source_age_ms == 612.5
    assert frame.playback_hold is True
    assert frame.lead_frames == 9
    assert frame.valid_horizon == 10
    assert frame.clamp_slots == 0
    assert frame.playout_seq == 123
    assert frame.consumer_session == 456


@pytest.mark.parametrize(
    "frame_count,valid_horizon,clamp_slots,error",
    [
        (2, WINDOW, 0, r"term1_local.*expected \(10,72\)"),
        (WINDOW + 1, WINDOW, 0, r"term1_local.*expected \(10,72\)"),
        (WINDOW, 2, 0, "valid_horizon=10"),
        (WINDOW, WINDOW, 8, "clamp_slots=0"),
    ],
)
def test_v4_live_reference_rejects_nonexact_or_clamped_windows(
    monkeypatch, frame_count, valid_horizon, clamp_slots, error
):
    policy = _make_policy(monkeypatch)
    fields = _live_fields(frame_count)
    fields["valid_horizon"] = np.array([valid_horizon], dtype=np.int64)
    fields["clamp_slots"] = np.array([clamp_slots], dtype=np.int64)

    with pytest.raises(ValueError, match=error):
        policy._frame_from_fields(fields)


def test_invalid_v4_window_is_dropped_without_replacing_held_reference(monkeypatch):
    policy = _make_policy(monkeypatch)
    held = _frame(epoch=4, consumer_session=22)
    policy.latest_live_ref = held
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    fields = _live_fields(frame_count=2)
    policy._zmq_inbound_queue.put_nowait(
        sonic_module.pack_pose_message(fields, topic="smpl_ref", version=4)
    )

    assert policy.poll_reference() is held
    assert policy.invalid_live_ref_messages == 1


def test_legacy_live_reference_keeps_short_window_compatibility(monkeypatch):
    policy = _make_policy(monkeypatch)
    fields = {
        "source_ready": np.array([True], dtype=bool),
        "source_stream_mode": np.array([1], dtype=np.int32),
        "source_calibration_ready": np.array([True], dtype=bool),
        "term1_local": np.zeros((2, 72), dtype=np.float32),
        "root_quat": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (2, 1),
        ),
        "wrist": np.zeros((2, 6), dtype=np.float32),
        "frame_index": np.array([41], dtype=np.int64),
    }
    fields["term1_local"][1, 0] = 9.0

    frame = policy._frame_from_fields(fields)

    assert frame.term1_local.shape == (WINDOW, 72)
    np.testing.assert_array_equal(
        frame.term1_local[2:],
        np.repeat(frame.term1_local[1:2], WINDOW - 2, axis=0),
    )
    assert frame.stream_epoch is None
    assert frame.playout_seq is None


def test_no_live_frame_still_returns_default_pose(monkeypatch):
    policy = _make_policy(monkeypatch)
    policy.poll_reference = lambda: None

    target = policy.inference_step(*_robot_observation())

    np.testing.assert_array_equal(target, policy.default_dof_pos)
    assert policy.session.calls == 0
    assert policy.policy_active is False
    assert policy.last_status == "waiting_for_live_smpl_ref"


def test_silent_or_explicitly_stale_stream_holds_reference_and_keeps_inference(
    monkeypatch,
):
    clock = [100.0]
    monkeypatch.setattr(sonic_module.time, "monotonic", lambda: clock[0])
    policy = _make_policy(monkeypatch)
    live = [_frame()]
    policy.poll_reference = lambda: live[0]
    policy.latest_live_ref_time = clock[0]

    fresh_target = policy.inference_step(*_robot_observation())
    assert policy.last_status == "live_reference"
    assert policy.session.calls == 1

    # The bridge process goes silent: retain the last full reference forever.
    clock[0] += 60.0
    held_target = policy.inference_step(*_robot_observation())
    assert policy.last_status == "stale_hold"
    assert policy.policy_active is True
    assert policy.session.calls == 2
    np.testing.assert_array_equal(held_target, fresh_target)
    assert not np.array_equal(held_target, policy.default_dof_pos)

    # A live bridge may also explicitly mark its repeated boundary window stale.
    live[0] = _frame(source_stale=True, source_age_ms=800.0, sequence=2)
    policy.latest_live_ref_time = clock[0]
    policy.inference_step(*_robot_observation())
    assert policy.last_status == "stale_hold"
    assert policy.session.calls == 3


def test_explicit_reset_ends_stale_hold_until_a_new_live_frame_arrives(monkeypatch):
    policy = _make_policy(monkeypatch)
    policy.poll_reference = lambda: policy.latest_live_ref
    policy.latest_live_ref = _frame(epoch=3, consumer_session=37)
    policy.consumer_session = 37
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    policy.inference_step(*_robot_observation())
    assert policy.session.calls == 1
    assert policy.last_status == "live_reference"

    policy.reset()

    assert policy.latest_live_ref is None
    assert policy.latest_live_ref_time == 0.0
    assert policy.stream_epoch is None
    request_id = policy.pending_reset_request_id
    assert request_id > 0
    assert not np.any(policy.last_action)
    assert not np.any(policy.base_ang_vel_history)
    assert not np.any(policy.joint_pos_history)
    assert not np.any(policy.joint_vel_history)
    assert not np.any(policy.action_history)
    assert not np.any(policy.gravity_history)

    waiting_target = policy.inference_step(*_robot_observation())
    np.testing.assert_array_equal(waiting_target, policy.default_dof_pos)
    assert policy.session.calls == 1
    assert policy.policy_active is False
    assert policy.last_status == "waiting_for_stream_reset"

    # A queued old/legacy frame cannot cross the explicit reset boundary.
    assert not _accept_live(
        policy,
        _frame(epoch=3, sequence=2, consumer_session=37),
    )
    assert not _accept_live(policy, _frame(epoch=4, sequence=3))
    assert policy.latest_live_ref is None

    assert _accept_live(
        policy,
        _frame(epoch=4, sequence=4, consumer_session=request_id),
    )
    resumed_target = policy.inference_step(*_robot_observation())
    assert policy.stream_epoch == 4
    assert policy.session.calls == 2
    assert policy.policy_active is True
    assert policy.last_status == "live_reference"
    assert not np.array_equal(resumed_target, policy.default_dof_pos)


def test_successful_inference_acks_epoch_and_playout_but_failure_does_not(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    control = policy._zmq_outbound_queue
    live = [_frame(epoch=9, playout_seq=44, consumer_session=73)]
    policy.poll_reference = lambda: live[0]
    policy.latest_live_ref_time = sonic_module.time.monotonic()

    policy.inference_step(*_robot_observation())

    ack = _decode_control(policy, control)
    assert int(ack["ack_consumer_session"][0]) == 73
    assert int(ack["ack_stream_epoch"][0]) == 9
    assert int(ack["ack_playout_seq"][0]) == 44

    class FailingSession:
        def run(self, _output_names, _inputs):
            raise RuntimeError("inference failed")

    policy.session = FailingSession()
    sent_before = len(control.messages)
    with pytest.raises(RuntimeError, match="inference failed"):
        policy.inference_step(*_robot_observation())
    assert len(control.messages) == sent_before


def test_reference_without_consumer_session_is_never_acked(monkeypatch):
    policy = _make_policy(monkeypatch)
    control = policy._zmq_outbound_queue
    live = [_frame(epoch=9, playout_seq=44, consumer_session=None)]
    policy.poll_reference = lambda: live[0]
    policy.latest_live_ref_time = sonic_module.time.monotonic()

    policy.inference_step(*_robot_observation())

    assert policy.session.calls == 1
    assert control.messages == []


def test_unavailable_control_socket_never_interrupts_policy_inference(monkeypatch):
    policy = _make_policy(monkeypatch)
    policy._zmq_outbound_queue = _FakeOutboundQueue(fail_put=True)
    live = [_frame(epoch=5, playout_seq=8, consumer_session=13)]
    policy.poll_reference = lambda: live[0]
    policy.latest_live_ref_time = sonic_module.time.monotonic()

    target = policy.inference_step(*_robot_observation())

    assert policy.last_status == "live_reference"
    assert policy.policy_active is True
    assert policy.control_send_failures == 1
    assert not np.array_equal(target, policy.default_dof_pos)


def test_reset_control_is_retried_with_same_session_until_bridge_echoes_it(
    monkeypatch,
):
    clock = [200.0]
    monkeypatch.setattr(sonic_module.time, "monotonic", lambda: clock[0])
    policy = _make_policy(monkeypatch)
    control = policy._zmq_outbound_queue
    policy.poll_reference = lambda: policy.latest_live_ref

    policy.reset()
    request_id = policy.pending_reset_request_id
    first_reset = _decode_control(policy, control)
    assert int(first_reset["reset_request_id"][0]) == request_id
    assert int(first_reset["reset_stream"][0]) == 1

    clock[0] += 0.49
    policy.inference_step(*_robot_observation())
    assert len(control.messages) == 1

    clock[0] += 0.02
    policy.inference_step(*_robot_observation())
    assert len(control.messages) == 2
    retry = _decode_control(policy, control)
    assert int(retry["reset_request_id"][0]) == request_id

    assert _accept_live(policy, _frame(consumer_session=request_id))
    policy.inference_step(*_robot_observation())
    assert policy.pending_reset_request_id == 0
    assert policy.last_status == "live_reference"


def test_dropped_reset_is_retried_after_outbound_queue_recovers(monkeypatch):
    clock = [300.0]
    monkeypatch.setattr(sonic_module.time, "monotonic", lambda: clock[0])
    policy = _make_policy(monkeypatch)
    outbound = _FakeOutboundQueue(fail_put=True)
    policy._zmq_outbound_queue = outbound
    policy.poll_reference = lambda: None

    policy.reset()
    request_id = policy.pending_reset_request_id
    assert outbound.messages == []
    assert policy.control_send_failures == 1

    outbound.fail_put = False
    clock[0] += 0.51
    policy.inference_step(*_robot_observation())

    retried = _decode_control(policy, outbound)
    assert int(retried["reset_request_id"][0]) == request_id
    assert int(retried["reset_stream"][0]) == 1
    assert policy.last_status == "waiting_for_stream_reset"


def test_bridge_restart_holds_active_ref_and_replays_current_consumer_session(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    control = policy._zmq_outbound_queue
    policy.poll_reference = lambda: policy.latest_live_ref
    active_session = 777
    old_ref = _frame(epoch=10, playout_seq=50, consumer_session=active_session)
    policy.consumer_session = active_session
    policy.latest_live_ref = old_ref
    policy.latest_live_ref_time = sonic_module.time.monotonic()

    # A restarted bridge advertises session 0; reject it and retain old_ref.
    restarted_ref = _frame(epoch=20, playout_seq=0, consumer_session=0)
    assert not policy._reference_allowed_after_reset(restarted_ref)
    assert policy.latest_live_ref is old_ref
    assert policy.pending_reset_request_id == active_session
    reset_message = _decode_control(policy, control)
    assert int(reset_message["reset_request_id"][0]) == active_session

    held_target = policy.inference_step(*_robot_observation())
    assert policy.last_status == "stale_hold"
    assert policy.policy_active is True
    assert not np.array_equal(held_target, policy.default_dof_pos)
    assert len(control.messages) == 2
    first = _decode_control(policy, control, 0)
    second = _decode_control(policy, control, 1)
    assert "reset_request_id" in first
    assert "ack_playout_seq" in second

    # The restarted bridge adopts the replayed session and starts a new epoch.
    assert _accept_live(
        policy,
        _frame(epoch=21, playout_seq=0, consumer_session=active_session),
    )
    policy.inference_step(*_robot_observation())
    assert policy.pending_reset_request_id == 0
    assert policy.stream_epoch == 21
    assert policy.last_status == "live_reference"


def test_malformed_inbound_is_dropped_without_losing_held_reference(monkeypatch):
    policy = _make_policy(monkeypatch)
    held = _frame(epoch=4, consumer_session=22)
    policy.latest_live_ref = held
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    policy._zmq_inbound_queue.put_nowait(b"smpl_ref" + b"not-json")

    assert policy.poll_reference() is held
    assert policy.invalid_live_ref_messages == 1


def test_zmq_io_thread_owns_create_receive_send_and_close(monkeypatch):
    main_thread_id = threading.get_ident()
    incoming = []

    fields = {
        "source_ready": np.array([True], dtype=bool),
        "source_stream_mode": np.array([1], dtype=np.int32),
        "source_calibration_ready": np.array([True], dtype=bool),
        "term1_local": np.zeros((WINDOW, 72), dtype=np.float32),
        "root_quat": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (WINDOW, 1),
        ),
        "wrist": np.zeros((WINDOW, 6), dtype=np.float32),
        "frame_index": np.array([10], dtype=np.int64),
        "stream_epoch": np.array([123], dtype=np.int64),
        "valid_horizon": np.array([WINDOW], dtype=np.int64),
        "clamp_slots": np.array([0], dtype=np.int64),
        "playout_seq": np.array([7], dtype=np.int64),
        "consumer_session": np.array([88], dtype=np.int64),
    }
    incoming.append(
        sonic_module.pack_pose_message(fields, topic="smpl_ref", version=4)
    )

    class ThreadOwnedSocket:
        def __init__(self, kind, owner):
            self.kind = kind
            self.owner = owner
            self.operation_threads = []
            self.sent = []
            self.closed = 0

        def _record(self):
            current = threading.get_ident()
            self.operation_threads.append(current)
            assert current == self.owner

        def setsockopt(self, *_args):
            self._record()

        def setsockopt_string(self, *_args):
            self._record()

        def connect(self, *_args):
            self._record()

        def recv(self, flags=0):
            self._record()
            assert flags == zmq.NOBLOCK
            if incoming:
                return incoming.pop(0)
            raise zmq.Again()

        def send(self, message, flags=0):
            self._record()
            assert flags == zmq.NOBLOCK
            self.sent.append(message)

        def close(self, linger=0):
            self._record()
            self.closed += 1

    class ThreadOwnedContext:
        def __init__(self):
            self.owner = threading.get_ident()
            self.sockets = {}
            self.term_threads = []

        def socket(self, kind):
            assert threading.get_ident() == self.owner
            socket = ThreadOwnedSocket(kind, self.owner)
            self.sockets[kind] = socket
            return socket

        def term(self):
            self.term_threads.append(threading.get_ident())
            assert self.term_threads[-1] == self.owner

    class ThreadOwnedPoller:
        def __init__(self):
            self.owner = threading.get_ident()
            self.socket = None
            self.unregister_threads = []

        def register(self, socket, _event):
            assert threading.get_ident() == self.owner
            self.socket = socket

        def poll(self, timeout=0):
            assert threading.get_ident() == self.owner
            if incoming:
                return [(self.socket, zmq.POLLIN)]
            time.sleep(min(timeout / 1000.0, 0.002))
            return []

        def unregister(self, socket):
            self.unregister_threads.append(threading.get_ident())
            assert self.unregister_threads[-1] == self.owner
            assert socket is self.socket

    contexts = []
    pollers = []

    def context_factory():
        context = ThreadOwnedContext()
        contexts.append(context)
        return context

    def poller_factory():
        poller = ThreadOwnedPoller()
        pollers.append(poller)
        return poller

    def fake_load_reference(self):
        self.ref_term1 = np.zeros((WINDOW, 72), dtype=np.float32)
        self.ref_root_quat = fields["root_quat"].copy()
        self.ref_wrist = np.zeros((WINDOW, 6), dtype=np.float32)
        self.ref_anchor_quat = self.ref_root_quat.copy()

    def fake_init_onnx(self):
        self.session = _FakeSession()
        self.input_info = SimpleNamespace(name="obs_dict", shape=(1, MODEL_INPUT_DIM))
        self.output_info = SimpleNamespace(name="action", shape=(1, NUM_JOINTS))
        self.input_buffer = np.zeros((1, MODEL_INPUT_DIM), dtype=np.float32)

    monkeypatch.setattr(SonicTeleopPolicy, "_load_stream_reference", fake_load_reference)
    monkeypatch.setattr(SonicTeleopPolicy, "_init_onnx", fake_init_onnx)
    monkeypatch.setattr(sonic_module.zmq, "Context", context_factory)
    monkeypatch.setattr(sonic_module.zmq, "Poller", poller_factory)

    policy = SonicTeleopPolicy(
        model_onnx_path="unused.onnx",
        stream_reference_npz="unused.npz",
        use_smpl_ref_zmq=True,
        require_live_reference=True,
        yaw_bias_rad=0.0,
    )
    policy.source_blend_duration_s = 0.0
    io_thread_id = policy._zmq_io_thread_id
    try:
        deadline = time.monotonic() + 1.0
        target = policy.default_dof_pos
        while time.monotonic() < deadline:
            target = policy.inference_step(*_robot_observation())
            if policy.last_status == "live_reference":
                break
            time.sleep(0.005)
        assert policy.last_status == "live_reference"
        assert not np.array_equal(target, policy.default_dof_pos)

        control_socket = contexts[0].sockets[zmq.PUSH]
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not control_socket.sent:
            time.sleep(0.005)
        assert control_socket.sent
        ack = sonic_module._decode_packed_message(
            control_socket.sent[-1], "smpl_ref_control"
        )
        assert int(ack["ack_consumer_session"][0]) == 88
    finally:
        policy.close()
        policy.close()

    assert io_thread_id is not None and io_thread_id != main_thread_id
    assert policy._zmq_thread is None
    context = contexts[0]
    ref_socket = context.sockets[zmq.SUB]
    control_socket = context.sockets[zmq.PUSH]
    assert ref_socket.closed == 1
    assert control_socket.closed == 1
    assert set(ref_socket.operation_threads) == {io_thread_id}
    assert set(control_socket.operation_threads) == {io_thread_id}
    assert context.term_threads == [io_thread_id]
    assert pollers[0].unregister_threads == [io_thread_id]


def test_close_called_by_owner_thread_never_self_joins(monkeypatch):
    policy = _make_policy(monkeypatch)

    class CurrentThreadProxy:
        ident = threading.get_ident()

        @staticmethod
        def is_alive():
            return True

        @staticmethod
        def join(_timeout=None):
            raise AssertionError("an owning ZMQ thread must never join itself")

    policy._zmq_thread = CurrentThreadProxy()
    policy.close()

    assert policy._zmq_stop_event.is_set()


def test_epoch_change_recaptures_heading_without_clearing_robot_history(monkeypatch):
    policy = _make_policy(monkeypatch)
    live = [_frame(epoch=1)]
    policy.poll_reference = lambda: live[0]
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    policy._active_reference()

    policy.yaw_aligned = True
    policy.yaw_offset = 1.25
    policy.last_action[:] = 0.25
    policy.base_ang_vel_history[:] = 1.0
    policy.joint_pos_history[:] = 2.0
    policy.joint_vel_history[:] = 3.0
    policy.action_history[:] = 4.0
    policy.gravity_history[:] = 5.0
    history_before = tuple(
        value.copy()
        for value in (
            policy.last_action,
            policy.base_ang_vel_history,
            policy.joint_pos_history,
            policy.joint_vel_history,
            policy.action_history,
            policy.gravity_history,
        )
    )

    live[0] = _frame(epoch=2, sequence=2)
    policy.latest_live_ref_time = sonic_module.time.monotonic()
    active, source, _ = policy._active_reference()
    assert active is live[0]
    assert source == "live"

    assert policy.stream_epoch == 2
    assert policy.yaw_aligned is False
    assert policy.yaw_offset == 0.0
    for actual, expected in zip(
        (
            policy.last_action,
            policy.base_ang_vel_history,
            policy.joint_pos_history,
            policy.joint_vel_history,
            policy.action_history,
            policy.gravity_history,
        ),
        history_before,
    ):
        np.testing.assert_array_equal(actual, expected)

    policy._build_model_input(live[0], *_robot_observation())
    assert policy.yaw_aligned is True
