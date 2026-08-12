import queue

import numpy as np

from bxi_example_py_elf3.inference.sonic import SonicTeleopPolicy
from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    _source_chunk_fields,
)
from bxi_example_py_elf3.sonic_pico.streamed_smpl_ref import (
    IncomingChunk,
    StreamedSmplRefMerger,
)
from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message


WINDOW = 10
TOPIC = "smpl_ref"


def _chunk(start: int, count: int = WINDOW) -> IncomingChunk:
    frame_ids = np.arange(start, start + count, dtype=np.int64)
    term1 = np.zeros((count, 72), dtype=np.float32)
    root = np.zeros((count, 4), dtype=np.float32)
    root[:, 0] = 1.0
    wrist = np.zeros((count, 6), dtype=np.float32)
    term1[:, 0] = frame_ids
    wrist[:, 0] = frame_ids
    return IncomingChunk(frame_ids, term1, root, wrist)


def _bare_policy() -> SonicTeleopPolicy:
    policy = SonicTeleopPolicy.__new__(SonicTeleopPolicy)
    policy._zmq_inbound_queue = queue.Queue(maxsize=64)
    policy.smpl_ref_zmq_topic = TOPIC
    policy.invalid_live_ref_messages = 0
    policy.stream_merger = StreamedSmplRefMerger()
    policy.source_stream_epoch = None
    policy.last_source_newest_frame = None
    policy.last_source_rx_mono = 0.0
    policy.source_chunk_messages = 0
    policy.source_chunk_duplicates = 0
    policy.source_chunk_restarts = 0
    policy.source_queue_drops = 0
    policy.has_seen_live_reference = False
    policy.live_reference_protocol = "none"
    policy.latest_live_ref = None
    policy.latest_live_ref_time = 0.0
    policy.live_sequence = 0
    policy.stream_epoch = None
    policy.live_ref_timeout_s = 0.5
    policy.yaw_aligned = False
    policy.yaw_offset = 0.0
    return policy


def _wire(start: int, *, epoch: int, received_ns: int = 1) -> bytes:
    fields = _source_chunk_fields(
        _chunk(start),
        source_stream_epoch=epoch,
        received_monotonic_ns=received_ns,
    )
    return pack_pose_message(fields, topic=TOPIC, version=5)


def test_bridge_source_schema_is_one_way_and_contains_no_ack_contract():
    fields = _source_chunk_fields(
        _chunk(100),
        source_stream_epoch=7,
        received_monotonic_ns=123,
    )
    assert int(fields["source_chunk"][0]) == 1
    np.testing.assert_array_equal(
        fields["frame_index"], np.arange(100, 110)
    )
    assert int(fields["valid_horizon"][0]) == WINDOW
    assert int(fields["clamp_slots"][0]) == 0
    assert not any(
        key.startswith("ack_")
        or key in {"playout_seq", "consumer_session", "reset_stream"}
        for key in fields
    )


def test_policy_consumes_rolling_chunks_in_order_with_nine_frame_overlap():
    policy = _bare_policy()
    for start, rx in ((0, 1.00), (1, 1.02), (2, 1.04)):
        policy._zmq_inbound_queue.put_nowait(
            (_wire(start, epoch=11), rx)
        )

    first = policy.poll_reference()
    assert first.frame_index == 0
    np.testing.assert_array_equal(
        first.term1_local[:, 0], np.arange(0, 10)
    )
    assert policy.stream_merger.advance_after_successful_tick()

    second = policy.poll_reference()
    assert second.frame_index == 1
    np.testing.assert_array_equal(
        second.term1_local[:, 0], np.arange(1, 11)
    )
    np.testing.assert_array_equal(
        first.term1_local[1:], second.term1_local[:-1]
    )


def test_duplicate_source_chunk_does_not_advance_or_refresh_freshness():
    policy = _bare_policy()
    fields = _source_chunk_fields(
        _chunk(0),
        source_stream_epoch=13,
        received_monotonic_ns=1,
    )
    assert policy._merge_source_fields(fields, 10.0)
    assert not policy._merge_source_fields(fields, 20.0)
    assert policy.last_source_rx_mono == 10.0
    assert policy.source_chunk_duplicates == 1
    assert policy.stream_merger.current_frame == 0


def test_source_epoch_restart_resets_atomically_to_new_complete_window():
    policy = _bare_policy()
    assert policy._merge_source_fields(
        _source_chunk_fields(
            _chunk(50),
            source_stream_epoch=1,
            received_monotonic_ns=1,
        ),
        1.0,
    )
    old_merger_epoch = policy.stream_merger.stream_epoch

    assert policy._merge_source_fields(
        _source_chunk_fields(
            _chunk(500),
            source_stream_epoch=2,
            received_monotonic_ns=2,
        ),
        2.0,
    )
    frame = policy.poll_reference()
    assert frame.frame_index == 500
    assert policy.stream_merger.stream_epoch != old_merger_epoch
    assert policy.source_stream_epoch == 2
