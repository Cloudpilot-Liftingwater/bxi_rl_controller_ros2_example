#!/usr/bin/env python3
"""Bridge official PICO manager ``pose`` stream to ELF3 ``smpl_ref`` stream.

The official ``gear_sonic/scripts/pico_manager_thread_server.py --manager`` sends
packed ZMQ messages on topic ``pose``.  For the ELF3 native _smpl.onnx deploy we
publish the same long-lived reference contract used by ``smpl_ref_bridge.py``:

    term1_local : float32 [10,72]   SMPL joints local, flattened per frame
    root_quat   : float32 [10,4]    SMPL root quaternion, wxyz
    wrist       : float32 [10,6]    ELF3 native wrist x/y/z, left then right

The bridge is intentionally only an adapter: PICO/SMPL normalization stays in
the official PICO manager, while the downstream SONIC policy consumes the
normalized reference tensors.  Rolling manager chunks are merged behind an
independent, continuous playback cursor.  Like the official C++ deployment,
the cursor advances by at most one frame per publish tick and waits at the
protected tail instead of clamping or jumping to the newest rolling window.
A local stop-and-wait ACK makes each successful policy inference the only event
that may advance that cursor, so PUB/SUB queue loss cannot skip reference time.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import secrets
import signal
import sys
import threading
import time
from typing import Any

import numpy as np
import zmq

try:
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float32
except Exception:  # pragma: no cover - allows non-ROS source-tree tooling
    rclpy = None
    SignalHandlerOptions = None
    Float32 = None

from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message


HEADER_SIZE = 1280
DTYPE_MAP = {
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"),
    "bool": np.dtype("?"),
}

# Current official PICO manager fills the 29-d joint_pos wrist slots using the
# original SONIC/G1 index convention.  We export them as ELF3 native wrist order:
#   l_wrist_x, l_wrist_y, l_wrist_z, r_wrist_x, r_wrist_y, r_wrist_z.
PICO_G1_LEGACY_WRIST_IDX = [23, 25, 27, 24, 26, 28]
ELF3_NATIVE_WRIST_IDX = [19, 20, 21, 26, 27, 28]
WINDOW = 10
HISTORY_FRAMES = 5
MAX_GAP_FRAMES = 200
DEFAULT_RATE_HZ = 50.0
MAX_STREAM_EPOCH = (1 << 63) - 1


PICO_BUTTON_FIELDS = (
    "left_trigger",
    "right_trigger",
    "left_grip",
    "right_grip",
)


def _field_scalar(fields: dict[str, np.ndarray], name: str) -> float | None:
    value = fields.get(name)
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float32).reshape(-1)
    if arr.size == 0:
        return None
    return float(arr[0])


def _field_int(fields: dict[str, np.ndarray], name: str) -> int | None:
    value = fields.get(name)
    if value is None:
        return None
    arr = np.asarray(value).reshape(-1)
    if arr.size == 0:
        return None
    return int(arr[0])


class PicoButtonRosPublisher:
    def __init__(self, enabled: bool = True):
        self.enabled = False
        self.node = None
        self.publishers = {}
        self._owns_rclpy = False
        if not enabled:
            return
        if rclpy is None or Float32 is None:
            print(
                "[pico->smpl_ref] WARN rclpy/std_msgs unavailable; "
                "PICO trigger ROS topics disabled",
                flush=True,
            )
            return
        if not rclpy.ok():
            init_kwargs = {"args": []}
            if SignalHandlerOptions is not None:
                # The bridge owns process signals.  Letting rclpy consume SIGINT/SIGTERM
                # shuts down ROS but leaves the bridge's long-running loop alive.
                init_kwargs["signal_handler_options"] = SignalHandlerOptions.NO
            rclpy.init(**init_kwargs)
            self._owns_rclpy = True
        self.node = rclpy.create_node("sonic_pico_button_bridge")
        self.publishers = {
            name: self.node.create_publisher(Float32, f"pico/{name}", 10)
            for name in PICO_BUTTON_FIELDS
        }
        self.enabled = True

    def publish(self, fields: dict[str, np.ndarray]) -> None:
        if not self.enabled or self.node is None:
            return
        for name, publisher in self.publishers.items():
            value = _field_scalar(fields, name)
            if value is None:
                continue
            msg = Float32()
            msg.data = value
            publisher.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_rclpy and rclpy is not None and rclpy.ok():
            rclpy.shutdown(uninstall_handlers=False)


def _install_stop_signal_handlers(
    stop_event: threading.Event,
) -> dict[signal.Signals, Any]:
    previous_handlers: dict[signal.Signals, Any] = {}

    def _request_stop(signum, _frame) -> None:
        if not stop_event.is_set():
            signal_name = signal.Signals(signum).name
            print(f"\n[pico->smpl_ref] received {signal_name}; stopping", flush=True)
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, _request_stop)
    return previous_handlers


def _restore_signal_handlers(previous_handlers: dict[signal.Signals, Any]) -> None:
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


@dataclass
class IncomingChunk:
    frame_indices: np.ndarray
    term1_local: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray


@dataclass
class MergeResult:
    did_catchup_reset: bool = False
    frame_offset_adjustment: int = 0
    frame_step: int = 1
    merge_deferred: bool = False


def _decode_packed_message(msg: bytes, topic: str) -> dict[str, np.ndarray] | None:
    prefix = topic.encode("utf-8")
    if not msg.startswith(prefix):
        return None
    payload = msg[len(prefix):]
    if len(payload) < HEADER_SIZE:
        return None

    raw_header = payload[:HEADER_SIZE].split(b"\x00", 1)[0]
    if not raw_header:
        return None
    header: dict[str, Any] = json.loads(raw_header.decode("utf-8"))
    data = memoryview(payload[HEADER_SIZE:])

    out: dict[str, np.ndarray] = {}
    offset = 0
    for field in header.get("fields", []):
        name = field["name"]
        dtype = DTYPE_MAP.get(field["dtype"])
        if dtype is None:
            raise ValueError(f"unsupported dtype for field {name}: {field['dtype']}")
        shape = tuple(int(x) for x in field.get("shape", []))
        count = int(np.prod(shape)) if shape else 1
        nbytes = dtype.itemsize * count
        if offset + nbytes > len(data):
            raise ValueError(f"field {name} exceeds payload bounds")
        arr = np.frombuffer(data[offset:offset + nbytes], dtype=dtype, count=count)
        out[name] = arr.reshape(shape).copy()
        offset += nbytes
    return out


def _as_frame_matrix(arr: np.ndarray, width: int, name: str) -> np.ndarray:
    arr = np.asarray(arr, dtype=np.float32)
    if arr.ndim == 1:
        arr = arr.reshape(1, width)
    elif arr.ndim > 2:
        arr = arr.reshape(arr.shape[0], -1)
    if arr.shape[1] != width:
        raise ValueError(f"{name} has shape {arr.shape}; expected (*,{width})")
    return np.ascontiguousarray(arr, dtype=np.float32)


def _extract_wrist_frames(joint_pos: np.ndarray, source: str) -> np.ndarray:
    jp = np.asarray(joint_pos, dtype=np.float32)
    if jp.ndim == 1:
        jp = jp.reshape(1, -1)
    if jp.shape[1] < 29:
        raise ValueError(f"joint_pos has shape {jp.shape}; expected at least 29 columns")

    if source == "pico_g1_legacy":
        idx = PICO_G1_LEGACY_WRIST_IDX
    elif source == "elf3_native":
        idx = ELF3_NATIVE_WRIST_IDX
    else:
        raise ValueError(f"unknown wrist source: {source}")
    return np.ascontiguousarray(jp[:, idx], dtype=np.float32)


def _parse_incoming_chunk(fields: dict[str, np.ndarray], wrist_source: str) -> IncomingChunk:
    missing = [
        k
        for k in ("frame_index", "smpl_joints", "body_quat_w", "joint_pos")
        if k not in fields
    ]
    if missing:
        raise ValueError(f"PICO pose message missing required fields: {missing}")

    smpl_joints = np.asarray(fields["smpl_joints"], dtype=np.float32)
    if smpl_joints.ndim == 2 and smpl_joints.shape[1] == 72:
        term1 = _as_frame_matrix(smpl_joints, 72, "smpl_joints")
    elif smpl_joints.ndim == 3 and smpl_joints.shape[1:] == (24, 3):
        term1 = _as_frame_matrix(
            smpl_joints.reshape(smpl_joints.shape[0], 72),
            72,
            "smpl_joints",
        )
    else:
        raise ValueError(f"smpl_joints has shape {smpl_joints.shape}; expected (N,24,3)")

    root_quat = _as_frame_matrix(fields["body_quat_w"], 4, "body_quat_w")
    wrist = _extract_wrist_frames(fields["joint_pos"], wrist_source)
    frame_indices = np.asarray(fields["frame_index"], dtype=np.int64).reshape(-1)

    n = term1.shape[0]
    if root_quat.shape[0] != n or wrist.shape[0] != n or frame_indices.shape[0] != n:
        raise ValueError(
            "PICO pose frame count mismatch: "
            f"frame_index={frame_indices.shape[0]} term1={term1.shape[0]} "
            f"root={root_quat.shape[0]} wrist={wrist.shape[0]}"
        )
    if n <= 0:
        raise ValueError("PICO pose message has zero frames")
    if n < WINDOW:
        raise ValueError(
            f"PICO pose message has {n} frames; need at least {WINDOW} "
            "(--num_frames_to_send 10)"
        )
    frame_deltas = np.diff(frame_indices)
    if np.any(frame_deltas != 1):
        raise ValueError(
            "frame_index must be consecutive with step 1: "
            f"{frame_indices.tolist()}"
        )

    return IncomingChunk(
        frame_indices=np.ascontiguousarray(frame_indices, dtype=np.int64),
        term1_local=term1,
        root_quat=root_quat,
        wrist=wrist,
    )


def _classify_frame_progress(newest_frame: int, previous_newest_frame: int | None) -> str:
    """Classify cross-message source progress without treating repeats as fresh."""
    if previous_newest_frame is None or newest_frame > previous_newest_frame:
        return "forward"
    if newest_frame == previous_newest_frame:
        return "duplicate"
    return "restart"


def _new_stream_epoch(previous: int | None = None) -> int:
    """Return a process-unique positive epoch that also changes on stream reset."""
    while True:
        epoch = secrets.randbelow(MAX_STREAM_EPOCH) + 1
        if epoch != previous:
            return epoch


@dataclass(frozen=True)
class PlayoutControl:
    ack_stream_epoch: int | None = None
    ack_playout_seq: int | None = None
    ack_consumer_session: int | None = None
    reset_request_id: int | None = None
    reset_stream: bool = False


def _parse_playout_control(fields: dict[str, np.ndarray]) -> PlayoutControl:
    return PlayoutControl(
        ack_stream_epoch=_field_int(fields, "ack_stream_epoch"),
        ack_playout_seq=_field_int(fields, "ack_playout_seq"),
        ack_consumer_session=_field_int(fields, "ack_consumer_session"),
        reset_request_id=_field_int(fields, "reset_request_id"),
        reset_stream=bool(_field_int(fields, "reset_stream") or 0),
    )


def _new_reset_request(
    control: PlayoutControl,
    consumer_session: int,
) -> int | None:
    request_id = control.reset_request_id
    if not control.reset_stream or request_id is None:
        return None
    if request_id <= consumer_session:
        return None
    return request_id


class AckGatedPlayout:
    """Stop-and-wait gate for lossless local bridge-to-policy playout."""

    def __init__(self, stream_epoch: int):
        self.stream_epoch = int(stream_epoch)
        self.playout_seq = 0
        self.awaiting_ack = False
        self.pending_epoch: int | None = None
        self.pending_seq: int | None = None
        self.pending_consumer_session: int | None = None
        self.accepted_acks = 0
        self.ignored_acks = 0
        self.publish_attempts = 0

    def synchronize_epoch(self, stream_epoch: int) -> bool:
        stream_epoch = int(stream_epoch)
        if stream_epoch == self.stream_epoch:
            return False
        self.stream_epoch = stream_epoch
        self.playout_seq = 0
        self.awaiting_ack = False
        self.pending_epoch = None
        self.pending_seq = None
        self.pending_consumer_session = None
        return True

    def decorate(
        self,
        smpl_ref: dict[str, np.ndarray],
        consumer_session: int,
    ) -> None:
        self.synchronize_epoch(int(smpl_ref["stream_epoch"][0]))
        smpl_ref["playout_seq"] = np.asarray([self.playout_seq], dtype=np.int64)
        smpl_ref["consumer_session"] = np.asarray([consumer_session], dtype=np.int64)

    def mark_published(
        self,
        stream_epoch: int,
        playout_seq: int,
        consumer_session: int,
    ) -> None:
        self.synchronize_epoch(stream_epoch)
        if int(playout_seq) != self.playout_seq:
            raise ValueError(
                f"published playout_seq={playout_seq}, expected {self.playout_seq}"
            )
        self.awaiting_ack = True
        self.pending_epoch = self.stream_epoch
        self.pending_seq = self.playout_seq
        self.pending_consumer_session = int(consumer_session)
        self.publish_attempts += 1

    def ack_matches(self, control: PlayoutControl) -> bool:
        matched = (
            self.awaiting_ack
            and control.ack_stream_epoch == self.pending_epoch
            and control.ack_playout_seq == self.pending_seq
            and control.ack_consumer_session == self.pending_consumer_session
        )
        if not matched and (
            control.ack_stream_epoch is not None
            or control.ack_playout_seq is not None
            or control.ack_consumer_session is not None
        ):
            self.ignored_acks += 1
        return bool(matched)

    def commit_ack(self) -> None:
        if not self.awaiting_ack or self.pending_seq is None:
            raise RuntimeError("cannot commit an ACK when no publication is pending")
        self.playout_seq = self.pending_seq + 1
        self.awaiting_ack = False
        self.pending_epoch = None
        self.pending_seq = None
        self.pending_consumer_session = None
        self.accepted_acks += 1


class LatestDeferredChunk:
    """Keep only the newest source chunk while an acknowledged window is owned."""

    def __init__(self) -> None:
        self.chunk: IncomingChunk | None = None
        self.received_mono: float | None = None
        self.requires_epoch_reset = False
        self.deferred_count = 0
        self.replaced_count = 0

    def store(
        self,
        chunk: IncomingChunk,
        received_mono: float | None,
        *,
        requires_epoch_reset: bool,
    ) -> None:
        if self.chunk is not None:
            self.replaced_count += 1
        self.chunk = chunk
        self.received_mono = received_mono
        self.requires_epoch_reset = (
            self.requires_epoch_reset or bool(requires_epoch_reset)
        )
        self.deferred_count += 1

    def pop(self) -> tuple[IncomingChunk, float | None, bool] | None:
        if self.chunk is None:
            return None
        item = (self.chunk, self.received_mono, self.requires_epoch_reset)
        self.chunk = None
        self.received_mono = None
        self.requires_epoch_reset = False
        return item

    def clear(self) -> None:
        self.chunk = None
        self.received_mono = None
        self.requires_epoch_reset = False


class StreamedSmplRefMerger:
    """Python port of C++ StreamedMotionMerger for live PICO SMPL refs."""

    def __init__(
        self,
        history_frames: int = HISTORY_FRAMES,
        max_gap_frames: int = MAX_GAP_FRAMES,
        catch_up_enabled: bool = True,
    ):
        self.history_frames = int(history_frames)
        self.max_gap_frames = int(max_gap_frames)
        self.catch_up_enabled = bool(catch_up_enabled)
        self.reset()

    def reset(self) -> None:
        self.stream_epoch = _new_stream_epoch(getattr(self, "stream_epoch", None))
        self.term1_local = np.zeros((0, 72), dtype=np.float32)
        self.root_quat = np.zeros((0, 4), dtype=np.float32)
        self.wrist = np.zeros((0, 6), dtype=np.float32)
        self.stream_window_start = 0
        self.current_frame = 0
        self.frame_step = 1
        self.total_merges = 0
        self.catchup_count = 0
        self.last_playback_held = True
        self.last_hold_reason = "waiting_for_window"
        self.last_published_local_frame = 0
        self.playback_hold_count = 0

    @property
    def timesteps(self) -> int:
        return int(self.term1_local.shape[0])

    def _calculate_frame_step(self, frame_indices: np.ndarray) -> int:
        if frame_indices.shape[0] < 2:
            return 1
        step = abs(int(frame_indices[1]) - int(frame_indices[0]))
        return step if step > 0 else 1

    def _calculate_sliding_window(
        self,
        incoming_frame_start: int,
        incoming_frame_end: int,
        frame_step: int,
    ) -> tuple[int, int, bool]:
        if self.timesteps <= 0:
            return incoming_frame_start, 0, True

        global_playback_frame = self.stream_window_start + frame_step * max(
            0, self.current_frame - self.history_frames
        )
        max_gap_frames = (
            self.max_gap_frames + self.history_frames
            if self.catch_up_enabled
            else sys.maxsize
        )
        stream_window_end = self.stream_window_start + frame_step * (self.timesteps - 1)

        if incoming_frame_start <= self.stream_window_start:
            return incoming_frame_start, 0, True
        if incoming_frame_end <= stream_window_end:
            return incoming_frame_start, 0, True

        desired_window_start = global_playback_frame
        tentative_window_start = min(desired_window_start, incoming_frame_start)
        delta_to_incoming = incoming_frame_start - tentative_window_start
        tentative_merge_dst = delta_to_incoming // frame_step if frame_step > 0 else 0
        large_gap_from_old = incoming_frame_start > stream_window_end + frame_step

        if tentative_merge_dst > max_gap_frames or large_gap_from_old:
            return incoming_frame_start, 0, True
        return tentative_window_start, tentative_merge_dst, False

    def merge(
        self,
        chunk: IncomingChunk,
        *,
        allow_epoch_change: bool = True,
    ) -> MergeResult:
        had_existing_stream = self.timesteps > 0
        frame_step = self._calculate_frame_step(chunk.frame_indices)
        incoming_frame_start = int(chunk.frame_indices[0])
        incoming_frame_end = int(chunk.frame_indices[-1])

        new_window_start, merge_dst_frame, did_catchup = self._calculate_sliding_window(
            incoming_frame_start,
            incoming_frame_end,
            frame_step,
        )

        if did_catchup and had_existing_stream and not allow_epoch_change:
            return MergeResult(frame_step=frame_step, merge_deferred=True)

        new_len = merge_dst_frame + int(chunk.frame_indices.shape[0])
        new_term1 = np.zeros((new_len, 72), dtype=np.float32)
        new_root = np.zeros((new_len, 4), dtype=np.float32)
        new_wrist = np.zeros((new_len, 6), dtype=np.float32)

        old_window_start = self.stream_window_start
        if merge_dst_frame > 0 and self.timesteps > 0:
            old_window_end = old_window_start + frame_step * self.timesteps
            need_start_global = new_window_start
            need_end_global = incoming_frame_start
            overlap_start_global = max(need_start_global, old_window_start)
            overlap_end_global = min(need_end_global, old_window_end)
            if overlap_start_global < overlap_end_global:
                start_offset_old = overlap_start_global - old_window_start
                start_offset_new = overlap_start_global - new_window_start
                overlap_span = overlap_end_global - overlap_start_global
                copy_src_idx = start_offset_old // frame_step if frame_step > 0 else 0
                copy_dst_idx = start_offset_new // frame_step if frame_step > 0 else 0
                copy_count = overlap_span // frame_step if frame_step > 0 else 0
                if copy_count > 0:
                    new_term1[copy_dst_idx:copy_dst_idx + copy_count] = (
                        self.term1_local[copy_src_idx:copy_src_idx + copy_count]
                    )
                    new_root[copy_dst_idx:copy_dst_idx + copy_count] = (
                        self.root_quat[copy_src_idx:copy_src_idx + copy_count]
                    )
                    new_wrist[copy_dst_idx:copy_dst_idx + copy_count] = (
                        self.wrist[copy_src_idx:copy_src_idx + copy_count]
                    )

        n_in = int(chunk.frame_indices.shape[0])
        new_term1[merge_dst_frame:merge_dst_frame + n_in] = chunk.term1_local
        new_root[merge_dst_frame:merge_dst_frame + n_in] = chunk.root_quat
        new_wrist[merge_dst_frame:merge_dst_frame + n_in] = chunk.wrist

        window_shift_ticks = new_window_start - old_window_start
        window_shift = window_shift_ticks // frame_step if frame_step > 0 else 0

        self.term1_local = new_term1
        self.root_quat = new_root
        self.wrist = new_wrist
        self.stream_window_start = new_window_start
        self.frame_step = frame_step
        self.total_merges += 1

        if did_catchup:
            self.current_frame = 0
            self.catchup_count += 1
            if had_existing_stream:
                # Official deploy reinitializes heading after the same catch-up.
                # The epoch lets the Python consumer detect that discontinuity.
                self.stream_epoch = _new_stream_epoch(self.stream_epoch)
            frame_offset_adjustment = 0
        else:
            self.current_frame = max(0, self.current_frame - window_shift)
            frame_offset_adjustment = window_shift

        return MergeResult(
            did_catchup_reset=did_catchup,
            frame_offset_adjustment=frame_offset_adjustment,
            frame_step=frame_step,
        )

    def build_smpl_ref(
        self,
        *,
        source_age_ms: float = 0.0,
        source_stale: bool = False,
    ) -> dict[str, np.ndarray] | None:
        """Gather one strict current-plus-nine window without moving the cursor.

        Call :meth:`advance_after_publish` only after the message was sent.  That
        preserves the official gather/send/advance order and prevents a failed
        publication from silently consuming a reference frame.
        """
        if self.timesteps < WINDOW:
            return None

        published_current = self.current_frame
        idx = published_current + np.arange(WINDOW, dtype=np.int64)
        current_global_frame = self.stream_window_start + published_current * self.frame_step
        newest_global_frame = self.stream_window_start + (self.timesteps - 1) * self.frame_step
        lead_frames = (newest_global_frame - current_global_frame) // self.frame_step
        candidate = published_current + 1
        held = candidate + WINDOW >= self.timesteps
        return {
            "term1_local": np.ascontiguousarray(self.term1_local[idx], dtype=np.float32),
            "root_quat": np.ascontiguousarray(self.root_quat[idx], dtype=np.float32),
            "wrist": np.ascontiguousarray(self.wrist[idx], dtype=np.float32),
            "frame_index": np.asarray([current_global_frame], dtype=np.int64),
            "newest_frame_index": np.asarray([newest_global_frame], dtype=np.int64),
            "lead_frames": np.asarray([lead_frames], dtype=np.int32),
            "valid_horizon": np.asarray([WINDOW], dtype=np.int32),
            "clamp_slots": np.asarray([0], dtype=np.int32),
            "stream_epoch": np.asarray([self.stream_epoch], dtype=np.int64),
            "source_age_ms": np.asarray([max(0.0, source_age_ms)], dtype=np.float32),
            "source_stale": np.asarray([int(source_stale)], dtype=np.uint8),
            "playback_hold": np.asarray([int(held)], dtype=np.uint8),
        }

    def advance_after_publish(self) -> bool:
        """Advance at most once, retaining one extra frame beyond the next window.

        Returns ``True`` when the cursor advanced and ``False`` when the official
        protected-tail guard held the current reference window.
        """
        if self.timesteps < WINDOW:
            return False

        self.last_published_local_frame = self.current_frame
        candidate = self.current_frame + 1
        advanced = candidate + WINDOW < self.timesteps
        if advanced:
            self.current_frame = candidate
            self.last_playback_held = False
            self.last_hold_reason = "advanced_after_publish"
        else:
            self.last_playback_held = True
            self.last_hold_reason = "protected_tail"
            self.playback_hold_count += 1
        return advanced


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pico-host", default="127.0.0.1")
    parser.add_argument("--pico-port", type=int, default=5556)
    parser.add_argument("--pico-topic", default="pose")
    parser.add_argument("--out-host", default="127.0.0.1")
    parser.add_argument("--out-port", type=int, default=5557)
    parser.add_argument("--out-topic", default="smpl_ref")
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=5558)
    parser.add_argument("--control-topic", default="smpl_ref_control")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="fixed smpl_ref publish rate in Hz")
    parser.add_argument("--history-frames", type=int, default=HISTORY_FRAMES)
    parser.add_argument("--max-gap-frames", type=int, default=MAX_GAP_FRAMES)
    parser.add_argument("--disable-catch-up", action="store_true")
    parser.add_argument(
        "--wrist-source",
        choices=("pico_g1_legacy", "elf3_native"),
        default="pico_g1_legacy",
        help="source layout of the incoming pose.joint_pos wrist fields",
    )
    parser.add_argument("--log-every", type=float, default=2.0)
    parser.add_argument(
        "--disable-ros-pico-topics",
        action="store_true",
        help="do not republish PICO trigger/grip fields as ROS Float32 topics",
    )
    parser.add_argument(
        "--stale-warning-seconds",
        type=float,
        default=0.5,
        help="mark the source stale when no advancing PICO chunk arrives for this long",
    )
    args = parser.parse_args()

    stop_event = threading.Event()
    previous_signal_handlers = _install_stop_signal_handlers(stop_event)

    button_pub = PicoButtonRosPublisher(
        enabled=not args.disable_ros_pico_topics
    )

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.RCVHWM, 1)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.pico_topic)
    sub.connect(f"tcp://{args.pico_host}:{args.pico_port}")

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 2)
    pub.bind(f"tcp://{args.out_host}:{args.out_port}")

    control_pull = ctx.socket(zmq.PULL)
    control_pull.setsockopt(zmq.LINGER, 0)
    control_pull.setsockopt(zmq.RCVHWM, 32)
    control_pull.bind(f"tcp://{args.control_host}:{args.control_port}")

    print(
        f"[pico->smpl_ref] SUB tcp://{args.pico_host}:{args.pico_port} topic='{args.pico_topic}'"
    )
    print(
        f"[pico->smpl_ref] PUB tcp://{args.out_host}:{args.out_port} topic='{args.out_topic}' "
        f"wrist_source={args.wrist_source}"
    )
    print(
        f"[pico->smpl_ref] PULL tcp://{args.control_host}:{args.control_port} "
        f"topic='{args.control_topic}' ACK-gated=True"
    )
    print(
        "[pico->smpl_ref] merger enabled "
        f"rate={args.rate}Hz history={args.history_frames} "
        f"max_gap={args.max_gap_frames} catch_up={not args.disable_catch_up} "
        "continuous_cursor=True tail_guard_extra=1",
        flush=True,
    )

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    poller.register(control_pull, zmq.POLLIN)
    merger = StreamedSmplRefMerger(
        history_frames=args.history_frames,
        max_gap_frames=args.max_gap_frames,
        catch_up_enabled=not args.disable_catch_up,
    )
    playout = AckGatedPlayout(merger.stream_epoch)
    period = 1.0 / args.rate
    next_tick = time.monotonic()
    last_log = 0.0
    sent = 0
    received = 0
    skipped = 0
    pending_fields: dict[str, np.ndarray] | None = None
    pending_received_mono: float | None = None
    deferred_source = LatestDeferredChunk()
    last_valid_input_mono: float | None = None
    last_valid_newest_frame: int | None = None
    duplicate_chunks = 0
    counter_restarts = 0
    control_resets = 0
    consumer_session = 0
    control_messages = 0
    invalid_controls = 0
    deferred_epoch_resets = 0
    stale_was_reported = False
    try:
        while not stop_event.is_set():
            now = time.time()
            mono_now = time.monotonic()
            timeout_ms = max(0, int((next_tick - mono_now) * 1000.0))
            events = dict(poller.poll(timeout=timeout_ms))
            if control_pull in events:
                while True:
                    try:
                        control_msg = control_pull.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    try:
                        control_fields = _decode_packed_message(
                            control_msg, args.control_topic
                        )
                    except Exception as exc:
                        invalid_controls += 1
                        print(
                            f"[pico->smpl_ref] ignored malformed control: {exc}",
                            flush=True,
                        )
                        continue
                    if control_fields is None:
                        invalid_controls += 1
                        continue
                    control_messages += 1
                    control = _parse_playout_control(control_fields)

                    if control.reset_stream:
                        request_id = _new_reset_request(control, consumer_session)
                        if request_id is None:
                            invalid_controls += 1
                        else:
                            previous_epoch = merger.stream_epoch
                            merger.reset()
                            playout.synchronize_epoch(merger.stream_epoch)
                            consumer_session = request_id
                            control_resets += 1
                            # Preserve last_valid_newest_frame deliberately: only
                            # a truly forward manager frame or a counter restart
                            # may repopulate the cleared stream after this reset.
                            last_valid_input_mono = None
                            pending_fields = None
                            pending_received_mono = None
                            deferred_source.clear()
                            stale_was_reported = False
                            print(
                                "[pico->smpl_ref] consumer stream reset; "
                                f"session={consumer_session} "
                                f"epoch={previous_epoch}->{merger.stream_epoch} "
                                f"last_manager_newest={last_valid_newest_frame}",
                                flush=True,
                            )
                            continue

                    if playout.ack_matches(control):
                        merger.advance_after_publish()
                        playout.commit_ack()

            if sub in events:
                while True:
                    try:
                        msg = sub.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    fields = _decode_packed_message(msg, args.pico_topic)
                    if fields is not None:
                        button_pub.publish(fields)
                        pending_fields = fields
                        pending_received_mono = time.monotonic()
                        received += 1

            if time.monotonic() < next_tick:
                continue

            chunk_to_process: IncomingChunk | None = None
            chunk_received_mono: float | None = None
            force_epoch_reset = False
            if pending_fields is not None:
                try:
                    chunk_to_process = _parse_incoming_chunk(
                        pending_fields, args.wrist_source
                    )
                    chunk_received_mono = pending_received_mono
                    force_epoch_reset = deferred_source.requires_epoch_reset
                except Exception as exc:
                    skipped += 1
                    print(f"[pico->smpl_ref] skipped invalid PICO pose: {exc}", flush=True)
                pending_fields = None
                pending_received_mono = None
            elif not playout.awaiting_ack:
                deferred_item = deferred_source.pop()
                if deferred_item is not None:
                    (
                        chunk_to_process,
                        chunk_received_mono,
                        force_epoch_reset,
                    ) = deferred_item

            if chunk_to_process is not None:
                try:
                    newest_frame = int(chunk_to_process.frame_indices[-1])
                    if playout.awaiting_ack and deferred_source.requires_epoch_reset:
                        deferred_source.store(
                            chunk_to_process,
                            chunk_received_mono,
                            requires_epoch_reset=True,
                        )
                    else:
                        progress = _classify_frame_progress(
                            newest_frame, last_valid_newest_frame
                        )
                        if force_epoch_reset:
                            previous_epoch = merger.stream_epoch
                            merger.reset()
                            playout.synchronize_epoch(merger.stream_epoch)
                            deferred_epoch_resets += 1
                            print(
                                "[pico->smpl_ref] applying ACK-deferred source reset; "
                                f"newest={newest_frame} "
                                f"previous={last_valid_newest_frame} "
                                f"epoch={previous_epoch}->{merger.stream_epoch}",
                                flush=True,
                            )
                            progress = "forward"

                        if progress == "duplicate":
                            duplicate_chunks += 1
                        elif progress == "restart" and playout.awaiting_ack:
                            deferred_source.store(
                                chunk_to_process,
                                chunk_received_mono,
                                requires_epoch_reset=True,
                            )
                        else:
                            if progress == "restart":
                                previous_epoch = merger.stream_epoch
                                merger.reset()
                                playout.synchronize_epoch(merger.stream_epoch)
                                counter_restarts += 1
                                print(
                                    "[pico->smpl_ref] PICO frame counter restarted; "
                                    f"newest={newest_frame} "
                                    f"previous={last_valid_newest_frame} "
                                    f"epoch={previous_epoch}->{merger.stream_epoch}",
                                    flush=True,
                                )

                            merge_result = merger.merge(
                                chunk_to_process,
                                allow_epoch_change=not playout.awaiting_ack,
                            )
                            if merge_result.merge_deferred:
                                deferred_source.store(
                                    chunk_to_process,
                                    chunk_received_mono,
                                    requires_epoch_reset=True,
                                )
                            else:
                                playout.synchronize_epoch(merger.stream_epoch)
                                deferred_source.clear()
                                last_valid_newest_frame = newest_frame
                                last_valid_input_mono = chunk_received_mono
                                stale_was_reported = False
                except Exception as exc:
                    skipped += 1
                    print(f"[pico->smpl_ref] skipped invalid PICO pose: {exc}", flush=True)

            tick_now = time.monotonic()
            input_age = (
                tick_now - last_valid_input_mono
                if last_valid_input_mono is not None
                else float("inf")
            )
            input_is_fresh = input_age <= args.stale_warning_seconds
            smpl_ref = merger.build_smpl_ref(
                source_age_ms=input_age * 1000.0,
                source_stale=not input_is_fresh,
            )
            if smpl_ref is not None:
                playout.decorate(smpl_ref, consumer_session)
                pub.send(pack_pose_message(smpl_ref, topic=args.out_topic, version=4))
                playout.mark_published(
                    int(smpl_ref["stream_epoch"][0]),
                    int(smpl_ref["playout_seq"][0]),
                    int(smpl_ref["consumer_session"][0]),
                )
                sent += 1

            if (
                last_valid_input_mono is not None
                and not input_is_fresh
                and not stale_was_reported
            ):
                print(
                    "[pico->smpl_ref] WARN PICO pose input stale; "
                    f"age_ms={input_age * 1000.0:.0f} received={received}. "
                    "continuing smpl_ref publication; consuming buffered frames then "
                    "holding the last complete window.",
                    flush=True,
                )
                stale_was_reported = True

            next_tick += period
            if next_tick < tick_now - period:
                next_tick = tick_now + period

            if now - last_log >= args.log_every:
                if smpl_ref is None:
                    print(
                        "[pico->smpl_ref] waiting for buffered PICO frames "
                        "reason='need 10 consecutive PICO frames' "
                        f"received={received} skipped={skipped} "
                        f"duplicates={duplicate_chunks} restarts={counter_restarts} "
                        f"session={consumer_session} acks={playout.accepted_acks} "
                        f"ack_ignored={playout.ignored_acks}",
                        flush=True,
                    )
                else:
                    print(
                        "[pico->smpl_ref] sent "
                        f"{sent} received={received} skipped={skipped} "
                        f"duplicates={duplicate_chunks} restarts={counter_restarts} "
                        f"playhead={int(smpl_ref['frame_index'][0])} "
                        f"newest={int(smpl_ref['newest_frame_index'][0])} "
                        f"lead={int(smpl_ref['lead_frames'][0])} "
                        f"valid_horizon={int(smpl_ref['valid_horizon'][0])} "
                        f"clamp_slots={int(smpl_ref['clamp_slots'][0])} "
                        f"epoch={int(smpl_ref['stream_epoch'][0])} "
                        f"seq={int(smpl_ref['playout_seq'][0])} "
                        f"session={int(smpl_ref['consumer_session'][0])} "
                        f"ack_pending={int(playout.awaiting_ack)} "
                        f"acks={playout.accepted_acks} ack_ignored={playout.ignored_acks} "
                        f"stale={int(smpl_ref['source_stale'][0])} "
                        f"hold={int(smpl_ref['playback_hold'][0])} "
                        f"hold_reason={merger.last_hold_reason} "
                        f"input_age_ms={input_age * 1000.0:.0f} "
                        f"local_published={merger.last_published_local_frame} "
                        f"local_next={merger.current_frame} "
                        f"T={merger.timesteps} window_start={merger.stream_window_start} "
                        f"catchups={merger.catchup_count} control={control_messages} "
                        f"control_invalid={invalid_controls} resets={control_resets} "
                        f"source_deferred={deferred_source.deferred_count} "
                        f"source_replaced={deferred_source.replaced_count} "
                        f"deferred_resets={deferred_epoch_resets} "
                        f"term1={smpl_ref['term1_local'].shape} "
                        f"root={smpl_ref['root_quat'].shape} wrist={smpl_ref['wrist'].shape}",
                        flush=True,
                    )
                last_log = now
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        cleanup_steps = (
            ("ROS button publisher", button_pub.close),
            ("PICO subscriber", lambda: sub.close(linger=0)),
            ("smpl_ref publisher", lambda: pub.close(linger=0)),
            ("smpl_ref control pull", lambda: control_pull.close(linger=0)),
            ("ZMQ context", ctx.term),
        )
        for resource_name, close_resource in cleanup_steps:
            try:
                close_resource()
            except Exception as exc:
                print(
                    f"[pico->smpl_ref] WARN failed to close {resource_name}: {exc}",
                    flush=True,
                )
        _restore_signal_handlers(previous_signal_handlers)
        print("[pico->smpl_ref] shutdown complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
