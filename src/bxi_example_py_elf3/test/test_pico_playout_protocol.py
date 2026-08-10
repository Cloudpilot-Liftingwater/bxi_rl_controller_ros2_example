import numpy as np

from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    AckGatedPlayout,
    AckPacedPublicationSchedule,
    IncomingChunk,
    LatestDeferredChunk,
    PlayoutControl,
    StreamedSmplRefMerger,
    _decode_packed_message,
    _new_reset_request,
    _parse_playout_control,
)
from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message


WINDOW = 10
CONTROL_TOPIC = "smpl_ref_control"


def _chunk(start: int, count: int = WINDOW) -> IncomingChunk:
    frame_ids = np.arange(start, start + count, dtype=np.int64)
    term1 = np.zeros((count, 72), dtype=np.float32)
    root = np.zeros((count, 4), dtype=np.float32)
    wrist = np.zeros((count, 6), dtype=np.float32)
    term1[:, 0] = frame_ids
    root[:, 1] = frame_ids
    wrist[:, 0] = frame_ids
    return IncomingChunk(frame_ids, term1, root, wrist)


def _ack(epoch: int, seq: int, session: int = 0) -> PlayoutControl:
    return PlayoutControl(
        ack_stream_epoch=epoch,
        ack_playout_seq=seq,
        ack_consumer_session=session,
    )


def _build_and_mark(
    merger: StreamedSmplRefMerger,
    gate: AckGatedPlayout,
    session: int = 0,
) -> dict[str, np.ndarray]:
    ref = merger.build_smpl_ref()
    assert ref is not None
    gate.decorate(ref, session)
    gate.mark_published(
        int(ref["stream_epoch"][0]),
        int(ref["playout_seq"][0]),
        int(ref["consumer_session"][0]),
    )
    return ref


def _commit_matching_ack(
    merger: StreamedSmplRefMerger,
    gate: AckGatedPlayout,
    control: PlayoutControl,
) -> bool:
    if not gate.ack_matches(control):
        return False
    merger.advance_after_publish()
    gate.commit_ack()
    return True


def test_control_wire_format_round_trip():
    wire = pack_pose_message(
        {
            "ack_stream_epoch": np.asarray([1234], dtype=np.int64),
            "ack_playout_seq": np.asarray([57], dtype=np.int64),
            "ack_consumer_session": np.asarray([2468], dtype=np.int64),
            "reset_request_id": np.asarray([9001], dtype=np.int64),
            "reset_stream": np.asarray([1], dtype=np.uint8),
        },
        topic=CONTROL_TOPIC,
        version=1,
    )
    fields = _decode_packed_message(wire, CONTROL_TOPIC)
    control = _parse_playout_control(fields)
    assert control == PlayoutControl(
        ack_stream_epoch=1234,
        ack_playout_seq=57,
        ack_consumer_session=2468,
        reset_request_id=9001,
        reset_stream=True,
    )


def test_no_ack_repeats_same_epoch_seq_and_window_without_advancing():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    merger.merge(_chunk(2))  # Enough buffered data to advance after an ACK.
    gate = AckGatedPlayout(merger.stream_epoch)

    first = _build_and_mark(merger, gate, session=41)
    second = _build_and_mark(merger, gate, session=41)

    assert int(first["frame_index"][0]) == int(second["frame_index"][0]) == 0
    assert int(first["playout_seq"][0]) == int(second["playout_seq"][0]) == 0
    assert int(first["stream_epoch"][0]) == int(second["stream_epoch"][0])
    assert int(first["consumer_session"][0]) == int(second["consumer_session"][0]) == 41
    np.testing.assert_array_equal(first["term1_local"], second["term1_local"])
    assert merger.current_frame == 0


def test_only_exact_ack_advances_once_and_old_or_duplicate_ack_is_ignored():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    merger.merge(_chunk(2))
    gate = AckGatedPlayout(merger.stream_epoch)
    ref = _build_and_mark(merger, gate)
    epoch = int(ref["stream_epoch"][0])

    assert not _commit_matching_ack(merger, gate, _ack(epoch + 1, 0))
    assert not _commit_matching_ack(merger, gate, _ack(epoch, 1))
    assert not _commit_matching_ack(merger, gate, _ack(epoch, 0, session=99))
    assert merger.current_frame == 0
    assert gate.playout_seq == 0

    assert _commit_matching_ack(merger, gate, _ack(epoch, 0))
    assert merger.current_frame == 1
    assert gate.playout_seq == 1

    assert not _commit_matching_ack(merger, gate, _ack(epoch, 0))
    next_ref = _build_and_mark(merger, gate)
    assert int(next_ref["frame_index"][0]) == 1
    assert int(next_ref["playout_seq"][0]) == 1


def test_tail_hold_ack_still_allocates_a_new_seq_for_same_frame():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(100))
    gate = AckGatedPlayout(merger.stream_epoch)
    first = _build_and_mark(merger, gate)
    epoch = int(first["stream_epoch"][0])

    assert int(first["playback_hold"][0]) == 1
    assert _commit_matching_ack(merger, gate, _ack(epoch, 0))
    assert merger.current_frame == 0
    assert gate.playout_seq == 1

    second = _build_and_mark(merger, gate)
    assert int(second["frame_index"][0]) == 100
    assert int(second["playout_seq"][0]) == 1


def test_epoch_reset_invalidates_old_ack_and_resets_seq():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    gate = AckGatedPlayout(merger.stream_epoch)
    old_ref = _build_and_mark(merger, gate)
    old_epoch = int(old_ref["stream_epoch"][0])

    merger.reset()
    gate.synchronize_epoch(merger.stream_epoch)
    merger.merge(_chunk(20))
    new_ref = _build_and_mark(merger, gate, session=123456)

    assert int(new_ref["stream_epoch"][0]) != old_epoch
    assert int(new_ref["playout_seq"][0]) == 0
    assert int(new_ref["consumer_session"][0]) == 123456
    assert not _commit_matching_ack(merger, gate, _ack(old_epoch, 0))
    assert merger.current_frame == 0


def test_large_gap_is_deferred_until_outstanding_ack_then_starts_new_epoch():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    gate = AckGatedPlayout(merger.stream_epoch)
    first = _build_and_mark(merger, gate)
    old_epoch = int(first["stream_epoch"][0])
    deferred = LatestDeferredChunk()

    far_chunk = _chunk(500)
    result = merger.merge(far_chunk, allow_epoch_change=not gate.awaiting_ack)
    assert result.merge_deferred
    deferred.store(far_chunk, 12.5, requires_epoch_reset=True)

    repeated = _build_and_mark(merger, gate)
    assert int(repeated["stream_epoch"][0]) == old_epoch
    assert int(repeated["playout_seq"][0]) == 0
    assert int(repeated["frame_index"][0]) == 0

    assert _commit_matching_ack(merger, gate, _ack(old_epoch, 0))
    item = deferred.pop()
    assert item is not None
    newest_chunk, _, requires_epoch_reset = item
    assert requires_epoch_reset
    catchup = merger.merge(newest_chunk, allow_epoch_change=True)
    assert catchup.did_catchup_reset
    assert gate.synchronize_epoch(merger.stream_epoch)

    new_ref = _build_and_mark(merger, gate)
    assert int(new_ref["stream_epoch"][0]) != old_epoch
    assert int(new_ref["playout_seq"][0]) == 0
    assert int(new_ref["frame_index"][0]) == 500


def test_multiple_deferred_chunks_keep_only_latest_rolling_window():
    deferred = LatestDeferredChunk()
    deferred.store(_chunk(100), 1.0, requires_epoch_reset=True)
    deferred.store(_chunk(200), 2.0, requires_epoch_reset=False)
    deferred.store(_chunk(300), 3.0, requires_epoch_reset=False)

    item = deferred.pop()
    assert item is not None
    chunk, received_mono, requires_epoch_reset = item
    assert int(chunk.frame_indices[0]) == 300
    assert received_mono == 3.0
    assert requires_epoch_reset
    assert deferred.deferred_count == 3
    assert deferred.replaced_count == 2
    assert deferred.pop() is None


def test_reset_request_must_be_enabled_positive_and_newer_than_session():
    assert _new_reset_request(PlayoutControl(), consumer_session=10) is None
    assert (
        _new_reset_request(
            PlayoutControl(reset_request_id=11, reset_stream=False),
            consumer_session=10,
        )
        is None
    )
    assert (
        _new_reset_request(
            PlayoutControl(reset_request_id=10, reset_stream=True),
            consumer_session=10,
        )
        is None
    )
    assert (
        _new_reset_request(
            PlayoutControl(reset_request_id=11, reset_stream=True),
            consumer_session=10,
        )
        == 11
    )


def test_early_ack_waits_for_rate_cap_and_resend_does_not_move_it():
    schedule = AckPacedPublicationSchedule(period=0.02)
    epoch = 7

    assert schedule.publication_due(
        0.0, stream_epoch=epoch, awaiting_ack=False, has_window=True
    ) == "initial"
    schedule.mark_published(0.0, "initial")
    schedule.record_ack(epoch)
    assert schedule.publication_due(
        0.019, stream_epoch=epoch, awaiting_ack=False, has_window=True
    ) is None
    assert schedule.publication_due(
        0.020, stream_epoch=epoch, awaiting_ack=False, has_window=True
    ) == "advance"

    schedule.reset_epoch()
    schedule.mark_published(0.0, "initial")
    assert schedule.publication_due(
        0.020, stream_epoch=epoch, awaiting_ack=True, has_window=True
    ) == "resend"
    schedule.mark_published(0.020, "resend")
    schedule.record_ack(epoch)
    assert schedule.publication_due(
        0.023, stream_epoch=epoch, awaiting_ack=False, has_window=True
    ) == "advance"


def test_unlucky_policy_phase_remains_near_fifty_hz_after_ack_priming():
    period = 0.02
    schedule = AckPacedPublicationSchedule(period=period)
    epoch = 11
    awaiting_ack = False
    sequence = -1
    policy_next_tick = 0.015
    policy_last_seen = -1
    ack_due = None
    new_publications = []

    # One-millisecond deterministic simulation: policy starts 15 ms after the
    # bridge and takes 8 ms, the phase that made the old fixed-tick loop run at
    # about 25 Hz.
    for millisecond in range(2001):
        now = millisecond / 1000.0
        if ack_due is not None and now + 1.0e-12 >= ack_due:
            awaiting_ack = False
            schedule.record_ack(epoch)
            ack_due = None

        publication = schedule.publication_due(
            now,
            stream_epoch=epoch,
            awaiting_ack=awaiting_ack,
            has_window=True,
        )
        if publication is not None:
            if publication != "resend":
                sequence += 1
                new_publications.append(now)
            schedule.mark_published(now, publication)
            awaiting_ack = True

        if now + 1.0e-12 >= policy_next_tick:
            if sequence > policy_last_seen:
                policy_last_seen = sequence
                ack_due = now + 0.008
            policy_next_tick += period

    assert len(new_publications) >= 98
    assert all(
        later - earlier >= period - 1.0e-9
        for earlier, later in zip(new_publications, new_publications[1:])
    )
