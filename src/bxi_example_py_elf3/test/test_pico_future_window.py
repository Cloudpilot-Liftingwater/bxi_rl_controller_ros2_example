from fractions import Fraction

import numpy as np
import pytest

from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    _parse_incoming_chunk,
)
from bxi_example_py_elf3.sonic_pico.streamed_smpl_ref import (
    IncomingChunk,
    StreamedSmplRefMerger,
    _classify_frame_progress,
)


WINDOW = 10


def _chunk(start: int, count: int = WINDOW) -> IncomingChunk:
    frame_ids = np.arange(start, start + count, dtype=np.int64)
    term1 = np.zeros((count, 72), dtype=np.float32)
    root = np.zeros((count, 4), dtype=np.float32)
    wrist = np.zeros((count, 6), dtype=np.float32)
    term1[:, 0] = frame_ids
    root[:, 1] = frame_ids
    wrist[:, 0] = frame_ids
    return IncomingChunk(frame_ids, term1, root, wrist)


def _assert_complete_window(
    ref: dict[str, np.ndarray],
    playhead: int,
    *,
    newest: int | None = None,
    held: bool | None = None,
) -> None:
    expected = np.arange(playhead, playhead + WINDOW, dtype=np.float32)
    np.testing.assert_array_equal(ref["term1_local"][:, 0], expected)
    np.testing.assert_array_equal(ref["root_quat"][:, 1], expected)
    np.testing.assert_array_equal(ref["wrist"][:, 0], expected)
    if newest is None:
        newest = playhead + WINDOW - 1
    assert int(ref["frame_index"][0]) == playhead
    assert int(ref["newest_frame_index"][0]) == newest
    assert int(ref["lead_frames"][0]) == newest - playhead
    assert int(ref["valid_horizon"][0]) == WINDOW
    assert int(ref["clamp_slots"][0]) == 0
    assert int(ref["stream_epoch"][0]) > 0
    if held is not None:
        assert bool(ref["playback_hold"][0]) is held


def _publish_tick(
    merger: StreamedSmplRefMerger,
    *,
    source_age_ms: float = 0.0,
    source_stale: bool = False,
) -> dict[str, np.ndarray] | None:
    """Model the policy's official gather/infer/advance ordering."""
    ref = merger.build_smpl_ref(
        source_age_ms=source_age_ms,
        source_stale=source_stale,
    )
    if ref is not None:
        merger.advance_after_successful_tick()
    return ref


def test_waits_for_ten_frames_then_emits_first_window_without_preadvance():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(100, count=WINDOW - 1))
    assert _publish_tick(merger) is None

    merger.merge(_chunk(100))
    _assert_complete_window(_publish_tick(merger), 100, held=True)


def test_official_tail_guard_builds_then_advances_and_keeps_one_extra_frame():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(100))
    _assert_complete_window(_publish_tick(merger), 100, newest=109, held=True)

    # Ten observation frames plus only one extra frame cannot advance yet.
    merger.merge(_chunk(101))
    _assert_complete_window(_publish_tick(merger), 100, newest=110, held=True)
    assert merger.current_frame == 0

    # With two extra frames, publish current first and advance only afterwards.
    merger.merge(_chunk(102))
    ref = merger.build_smpl_ref()
    _assert_complete_window(ref, 100, newest=111, held=False)
    assert merger.current_frame == 0  # A failed inference must not consume it.
    assert merger.advance_after_successful_tick()
    assert merger.current_frame == 1
    _assert_complete_window(_publish_tick(merger), 101, newest=111, held=True)


def test_rate_mismatch_keeps_global_playhead_continuous_across_rolling_merges():
    """Simulate 120 s of 49.5 Hz input and 50 Hz official-style playback."""
    merger = StreamedSmplRefMerger()
    source_start = 0
    merger.merge(_chunk(source_start))
    next_source_time = Fraction(2, 99)
    source_period = Fraction(2, 99)
    held_windows = 0
    previous_playhead = None

    for output_tick in range(120 * 50):
        output_time = Fraction(output_tick, 50)
        while next_source_time <= output_time:
            source_start += 1
            merger.merge(_chunk(source_start))
            next_source_time += source_period

        ref = _publish_tick(merger)
        playhead = int(ref["frame_index"][0])
        _assert_complete_window(
            ref,
            playhead,
            newest=source_start + WINDOW - 1,
        )
        if previous_playhead is not None:
            assert playhead - previous_playhead in (0, 1)
        if playhead == previous_playhead:
            held_windows += 1
        previous_playhead = playhead

    # 49.5 Hz versus 50 Hz should hold only about 0.5 tick/s, not repeat
    # roughly half the windows as the removed ACK gate did.
    assert 55 <= held_windows <= 70
    assert previous_playhead / 120.0 >= 49.4
    assert int(ref["lead_frames"][0]) >= WINDOW


def test_burst_is_played_sequentially_and_large_gap_resets_atomically():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    first_epoch = merger.stream_epoch
    _assert_complete_window(_publish_tick(merger), 0, newest=9, held=True)

    # Intermediate rolling chunks may be discarded by latest-value transport.
    merger.merge(_chunk(5))
    playheads = []
    for expected in (0, 1, 2, 3, 4, 4):
        ref = _publish_tick(merger)
        playheads.append(int(ref["frame_index"][0]))
        _assert_complete_window(ref, expected, newest=14)
    assert playheads == [0, 1, 2, 3, 4, 4]

    # A gap larger than the rolling-window overlap is a clean catch-up reset.
    result = merger.merge(_chunk(500))
    assert result.did_catchup_reset
    assert merger.stream_epoch != first_epoch
    _assert_complete_window(_publish_tick(merger), 500, newest=509, held=True)


def test_stale_source_consumes_buffer_then_holds_and_keeps_publishing():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(0))
    _publish_tick(merger)
    merger.merge(_chunk(5))

    refs = [
        _publish_tick(merger, source_age_ms=600.0 + tick * 20.0, source_stale=True)
        for tick in range(6)
    ]
    assert [int(ref["frame_index"][0]) for ref in refs] == [0, 1, 2, 3, 4, 4]
    assert all(int(ref["source_stale"][0]) == 1 for ref in refs)
    assert all(float(ref["source_age_ms"][0]) >= 600.0 for ref in refs)
    assert all(ref["term1_local"].shape == (WINDOW, 72) for ref in refs)
    assert int(refs[-1]["playback_hold"][0]) == 1


def test_explicit_counter_restart_changes_stream_epoch():
    merger = StreamedSmplRefMerger()
    merger.merge(_chunk(50))
    old_epoch = int(_publish_tick(merger)["stream_epoch"][0])

    merger.reset()
    merger.merge(_chunk(0))
    new_epoch = int(_publish_tick(merger)["stream_epoch"][0])
    assert new_epoch != old_epoch


@pytest.mark.parametrize(
    "newest, previous, expected",
    [
        (9, None, "forward"),
        (10, 9, "forward"),
        (9, 9, "duplicate"),
        (9, 100, "restart"),
    ],
)
def test_cross_message_progress_does_not_treat_duplicate_window_as_fresh(
    newest: int, previous: int | None, expected: str
):
    assert _classify_frame_progress(newest, previous) == expected


@pytest.mark.parametrize(
    "frame_indices, expected_error",
    [
        (np.arange(WINDOW - 1, dtype=np.int64), "need at least 10"),
        (
            np.array([0, 1, 2, 4, 5, 6, 7, 8, 9, 10], dtype=np.int64),
            "must be consecutive",
        ),
    ],
)
def test_parser_rejects_incomplete_or_nonconsecutive_windows(
    frame_indices: np.ndarray, expected_error: str
):
    frame_count = frame_indices.size
    fields = {
        "frame_index": frame_indices,
        "smpl_joints": np.zeros((frame_count, 24, 3), dtype=np.float32),
        "body_quat_w": np.zeros((frame_count, 4), dtype=np.float32),
        "joint_pos": np.zeros((frame_count, 29), dtype=np.float32),
        "wrist": np.zeros((frame_count, 6), dtype=np.float32),
        "wrist_layout_version": np.array([1], dtype=np.int32),
    }

    with pytest.raises(ValueError, match=expected_error):
        _parse_incoming_chunk(fields)
