import json
import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from bxi_example_py_elf3.inference.sonic import (
    MODEL_INPUT_DIM,
    NUM_JOINTS,
    WINDOW,
    SonicTeleopPolicy,
)
from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    _source_chunk_fields,
)
from bxi_example_py_elf3.sonic_pico.streamed_smpl_ref import (
    IncomingChunk,
)
from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message


class _FakeSession:
    def __init__(self):
        self.calls = 0
        self.fail = False
        self.inputs = []

    def run(self, _output_names, inputs):
        self.calls += 1
        self.inputs.append(next(iter(inputs.values())).copy())
        if self.fail:
            raise RuntimeError("synthetic inference failure")
        return [np.ones((1, NUM_JOINTS), dtype=np.float32)]


def _make_policy(monkeypatch, *, require_live_reference=True):
    def fake_load_reference(self):
        identity = np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (WINDOW, 1),
        )
        self.ref_term1 = np.zeros((WINDOW + 4, 72), dtype=np.float32)
        self.ref_root_quat = np.tile(identity[:1], (WINDOW + 4, 1))
        self.ref_wrist = np.zeros((WINDOW + 4, 6), dtype=np.float32)
        self.ref_anchor_quat = self.ref_root_quat.copy()

        self.stand_term1 = np.zeros((WINDOW, 72), dtype=np.float32)
        self.stand_term1[:, 0] = 42.0
        self.stand_root_quat = identity.copy()
        self.stand_wrist = np.zeros((WINDOW, 6), dtype=np.float32)
        self.stand_anchor_quat = identity.copy()

    def fake_init_onnx(self):
        self.session = _FakeSession()
        self.input_info = SimpleNamespace(
            name="obs_dict", shape=(1, MODEL_INPUT_DIM)
        )
        self.output_info = SimpleNamespace(
            name="action", shape=(1, NUM_JOINTS)
        )
        self.input_buffer = np.zeros(
            (1, MODEL_INPUT_DIM), dtype=np.float32
        )

    def fake_init_zmq(self):
        self.zmq_context = None
        self.zmq_socket = None
        self.zmq_poller = None
        self._zmq_inbound_queue = queue.Queue(maxsize=64)
        self._zmq_stop_event = threading.Event()
        self._zmq_ready_event = threading.Event()
        self._zmq_ready_event.set()
        self._zmq_thread = None
        self._zmq_io_thread_id = None
        self._zmq_start_error = None

    monkeypatch.setattr(
        SonicTeleopPolicy, "_load_stream_reference", fake_load_reference
    )
    monkeypatch.setattr(SonicTeleopPolicy, "_init_onnx", fake_init_onnx)
    monkeypatch.setattr(SonicTeleopPolicy, "_init_zmq", fake_init_zmq)
    return SonicTeleopPolicy(
        model_onnx_path="unused.onnx",
        stream_reference_npz="unused.npz",
        stand_reference_npz="unused-stand.npz",
        use_smpl_ref_zmq=False,
        require_live_reference=require_live_reference,
        yaw_bias_rad=0.0,
    )


def _chunk(start: int) -> IncomingChunk:
    frames = np.arange(start, start + WINDOW, dtype=np.int64)
    term1 = np.zeros((WINDOW, 72), dtype=np.float32)
    term1[:, 0] = frames
    root = np.zeros((WINDOW, 4), dtype=np.float32)
    root[:, 0] = 1.0
    wrist = np.zeros((WINDOW, 6), dtype=np.float32)
    wrist[:, 0] = frames
    return IncomingChunk(frames, term1, root, wrist)


def _enqueue_source(
    policy: SonicTeleopPolicy,
    starts,
    *,
    epoch: int = 1,
) -> None:
    now = time.monotonic()
    for offset, start in enumerate(starts):
        fields = _source_chunk_fields(
            _chunk(start),
            source_stream_epoch=epoch,
            received_monotonic_ns=int(
                (now + offset * 0.001) * 1.0e9
            ),
        )
        policy._zmq_inbound_queue.put_nowait(
            (
                pack_pose_message(
                    fields,
                    topic=policy.smpl_ref_zmq_topic,
                    version=5,
                ),
                now + offset * 0.001,
            )
        )


def _observation():
    return (
        np.zeros(NUM_JOINTS, dtype=np.float32),
        np.zeros(NUM_JOINTS, dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float32),
    )


def _legacy_window_fields(*, clamp_slots=0, frame_count=WINDOW):
    return {
        "term1_local": np.zeros(
            (frame_count, 72), dtype=np.float32
        ),
        "root_quat": np.tile(
            np.array(
                [[1.0, 0.0, 0.0, 0.0]], dtype=np.float32
            ),
            (frame_count, 1),
        ),
        "wrist": np.zeros((frame_count, 6), dtype=np.float32),
        "frame_index": np.asarray([100], dtype=np.int64),
        "newest_frame_index": np.asarray([109], dtype=np.int64),
        "stream_epoch": np.asarray([7], dtype=np.int64),
        "valid_horizon": np.asarray([WINDOW], dtype=np.int32),
        "clamp_slots": np.asarray([clamp_slots], dtype=np.int32),
    }


def test_before_first_live_uses_recorded_stand_reference_through_onnx(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    target = policy.inference_step(*_observation())

    assert policy.session.calls == 1
    assert policy.active_reference_kind == "standby"
    assert policy.last_status == "standby_reference"
    assert float(policy.session.inputs[-1][0, 0]) == 42.0
    assert not np.array_equal(target, policy.default_dof_pos)


def test_preheat_uses_stand_without_polling_or_consuming_live_reference(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))
    queued_before = policy._zmq_inbound_queue.qsize()

    for _ in range(policy.obs_history_len * 2):
        policy.preheat_step(*_observation())

    assert policy.session.calls == policy.obs_history_len * 2
    assert policy._zmq_inbound_queue.qsize() == queued_before
    assert not policy.has_seen_live_reference
    assert policy.stream_merger.current_frame == 0
    assert float(policy.session.inputs[-1][0, 0]) == 42.0

    policy.inference_step(*_observation())
    assert policy.has_seen_live_reference
    assert policy.active_reference_kind == "source_chunk"
    assert policy.stream_merger.current_frame == 1
    assert float(policy.session.inputs[-1][0, 0]) == 0.0


def test_successful_policy_ticks_own_and_advance_the_source_cursor(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))

    policy.inference_step(*_observation())
    assert policy.active_reference_kind == "source_chunk"
    assert float(policy.session.inputs[-1][0, 0]) == 0.0
    assert policy.stream_merger.current_frame == 1

    policy.inference_step(*_observation())
    assert float(policy.session.inputs[-1][0, 0]) == 1.0
    assert policy.stream_merger.current_frame == 1


def test_failed_inference_does_not_advance_reference_cursor(monkeypatch):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))
    policy.session.fail = True

    with pytest.raises(RuntimeError, match="synthetic inference failure"):
        policy.inference_step(*_observation())
    assert policy.stream_merger.current_frame == 0

    policy.session.fail = False
    policy.inference_step(*_observation())
    assert policy.stream_merger.current_frame == 1


def test_disconnect_consumes_buffer_then_holds_last_complete_window(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, range(6))

    observed_starts = []
    for _ in range(8):
        policy.inference_step(*_observation())
        observed_starts.append(
            int(policy.session.inputs[-1][0, 0])
        )

    assert observed_starts == [0, 1, 2, 3, 4, 4, 4, 4]
    policy.last_source_rx_mono = time.monotonic() - 1.0
    policy.inference_step(*_observation())
    assert policy.last_status == "stale_hold"
    assert policy.stream_merger.current_frame == 4


def test_explicit_controller_reset_returns_to_stand_until_new_live(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))
    policy.inference_step(*_observation())
    assert policy.has_seen_live_reference

    policy.reset()
    policy.inference_step(*_observation())
    assert not policy.has_seen_live_reference
    assert policy.active_reference_kind == "standby"
    assert float(policy.session.inputs[-1][0, 0]) == 42.0


def test_legacy_exact_window_replay_still_works_without_ack(monkeypatch):
    policy = _make_policy(monkeypatch)
    fields = _legacy_window_fields()
    now = time.monotonic()
    policy._zmq_inbound_queue.put_nowait(
        (
            pack_pose_message(
                fields,
                topic=policy.smpl_ref_zmq_topic,
                version=4,
            ),
            now,
        )
    )

    frame = policy.poll_reference()
    assert frame.frame_index == 100
    assert policy.live_reference_protocol == "legacy_window"
    assert policy.stream_merger.current_frame == 0


def test_invalid_clamped_live_window_is_dropped_without_replacing_hold(
    monkeypatch,
):
    policy = _make_policy(monkeypatch)
    now = time.monotonic()
    valid = pack_pose_message(
        _legacy_window_fields(),
        topic=policy.smpl_ref_zmq_topic,
        version=4,
    )
    invalid = pack_pose_message(
        _legacy_window_fields(clamp_slots=1),
        topic=policy.smpl_ref_zmq_topic,
        version=4,
    )
    policy._zmq_inbound_queue.put_nowait((valid, now))
    held = policy.poll_reference()
    policy._zmq_inbound_queue.put_nowait((invalid, now + 0.01))

    assert policy.poll_reference() is held
    assert policy.invalid_live_ref_messages == 1


def test_offline_npz_mode_keeps_its_local_cursor(monkeypatch):
    policy = _make_policy(
        monkeypatch, require_live_reference=False
    )
    policy.inference_step(*_observation())
    policy.inference_step(*_observation())
    assert policy.active_reference_kind == "offline"
    assert policy.motion_cursor == 2
    assert policy.stream_merger.current_frame == 0


def test_playback_telemetry_observes_success_hold_and_failure(monkeypatch, capsys):
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))

    policy.preheat_step(*_observation())
    assert policy.successful_inference_tick == 0
    assert policy.latest_playback_telemetry is None

    policy.session.fail = True
    with pytest.raises(RuntimeError, match="synthetic inference failure"):
        policy.inference_step(*_observation())
    assert policy.stream_merger.current_frame == 0
    assert policy.successful_inference_tick == 0
    assert policy.latest_playback_telemetry is None

    policy.session.fail = False
    policy.inference_step(*_observation())
    first = policy.latest_playback_telemetry
    assert first.frame_index == 0
    assert first.newest_frame_index == 11
    assert first.lead_frames == 11
    assert first.playback_hold is False
    assert first.catchup_count == policy.stream_merger.catchup_count
    assert first.stream_epoch == policy.stream_epoch
    assert first.successful_inference_tick == 1
    assert policy.stream_merger.current_frame == 1

    same_snapshot = policy.latest_playback_telemetry
    assert same_snapshot is first
    assert policy.stream_merger.current_frame == 1

    policy.inference_step(*_observation())
    second = policy.latest_playback_telemetry
    assert second.frame_index == 1
    assert second.newest_frame_index == 11
    assert second.lead_frames == 10
    assert second.playback_hold is True
    assert second.successful_inference_tick == 2
    assert policy.stream_merger.current_frame == 1
    assert "sonic-playback-telemetry" not in capsys.readouterr().out

    policy.reset()
    assert policy.latest_playback_telemetry is None
    assert policy.successful_inference_tick == 2


def test_playback_telemetry_logging_is_opt_in_json(monkeypatch, capsys):
    monkeypatch.setenv("BXI_SONIC_TELEMETRY_LOG_EVERY", "2")
    policy = _make_policy(monkeypatch)
    _enqueue_source(policy, (0, 1, 2))

    policy.inference_step(*_observation())
    assert "sonic-playback-telemetry" not in capsys.readouterr().out

    policy.inference_step(*_observation())
    lines = [
        line
        for line in capsys.readouterr().out.splitlines()
        if line.startswith("[sonic-playback-telemetry] ")
    ]
    assert len(lines) == 1
    payload = json.loads(lines[0].split(" ", 1)[1])
    assert payload == {
        "catchup_count": policy.stream_merger.catchup_count,
        "frame_index": 1,
        "lead_frames": 10,
        "newest_frame_index": 11,
        "playback_hold": True,
        "stream_epoch": policy.stream_epoch,
        "successful_inference_tick": 2,
    }
