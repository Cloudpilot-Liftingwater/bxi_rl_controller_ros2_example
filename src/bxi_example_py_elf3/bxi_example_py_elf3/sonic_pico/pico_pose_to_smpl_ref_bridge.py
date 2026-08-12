#!/usr/bin/env python3
"""Convert the official PICO ``pose`` stream to ELF3 source chunks.

The official ``gear_sonic/scripts/pico_manager_thread_server.py --manager`` sends
packed ZMQ messages on topic ``pose``.  For the ELF3 native _smpl.onnx deploy we
forward complete rolling chunks in the ELF3-native layout:

    term1_local : float32 [N,72]   SMPL joints local, flattened per frame
    root_quat   : float32 [N,4]    SMPL root quaternion, wxyz
    wrist       : float32 [N,6]    ELF3 native wrist x/y/z, left then right

The bridge is intentionally only a stateless format/ELF3-layout adapter.  It
owns no playback clock, cursor or ACK channel.  The downstream SONIC policy
owns ``current_frame`` and advances it only after a successful control tick,
matching the official C++ deployment.
"""

from __future__ import annotations

import argparse
import json
import signal
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

from bxi_example_py_elf3.sonic_pico.streamed_smpl_ref import (
    IncomingChunk,
    WINDOW,
    _classify_frame_progress,
    _new_stream_epoch,
)
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



def _source_chunk_fields(
    chunk: IncomingChunk,
    *,
    source_stream_epoch: int,
    received_monotonic_ns: int,
) -> dict[str, np.ndarray]:
    """Build the one-way canonical chunk consumed by the policy merger."""
    return {
        "source_chunk": np.asarray([1], dtype=np.uint8),
        "source_stream_epoch": np.asarray(
            [source_stream_epoch], dtype=np.int64
        ),
        "source_received_monotonic_ns": np.asarray(
            [received_monotonic_ns], dtype=np.int64
        ),
        "frame_index": np.ascontiguousarray(
            chunk.frame_indices, dtype=np.int64
        ),
        "term1_local": np.ascontiguousarray(
            chunk.term1_local, dtype=np.float32
        ),
        "root_quat": np.ascontiguousarray(
            chunk.root_quat, dtype=np.float32
        ),
        "wrist": np.ascontiguousarray(chunk.wrist, dtype=np.float32),
        "valid_horizon": np.asarray([WINDOW], dtype=np.int32),
        "clamp_slots": np.asarray([0], dtype=np.int32),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pico-host", default="127.0.0.1")
    parser.add_argument("--pico-port", type=int, default=5556)
    parser.add_argument("--pico-topic", default="pose")
    parser.add_argument("--out-host", default="127.0.0.1")
    parser.add_argument("--out-port", type=int, default=5557)
    parser.add_argument("--out-topic", default="smpl_ref")
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
        help="warn when no advancing PICO source chunk arrives for this long",
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
    sub.setsockopt(zmq.RCVHWM, 64)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.pico_topic)
    sub.connect(f"tcp://{args.pico_host}:{args.pico_port}")

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 8)
    pub.bind(f"tcp://{args.out_host}:{args.out_port}")

    print(
        f"[pico->smpl_ref] SUB tcp://{args.pico_host}:{args.pico_port} "
        f"topic='{args.pico_topic}'"
    )
    print(
        f"[pico->smpl_ref] PUB tcp://{args.out_host}:{args.out_port} "
        f"topic='{args.out_topic}' wrist_source={args.wrist_source}"
    )
    print(
        "[pico->smpl_ref] one-way source chunks enabled "
        "playback_owner=policy ack_channel=none fixed_publish_clock=none",
        flush=True,
    )

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    source_stream_epoch = _new_stream_epoch()
    last_valid_newest_frame: int | None = None
    last_valid_input_mono: float | None = None
    stale_was_reported = False
    received = 0
    sent = 0
    skipped = 0
    duplicate_chunks = 0
    counter_restarts = 0
    send_dropped = 0
    last_log = 0.0
    started_mono = time.monotonic()

    try:
        while not stop_event.is_set():
            events = dict(poller.poll(timeout=100))
            if sub in events:
                while True:
                    try:
                        msg = sub.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break

                    received += 1
                    received_mono = time.monotonic()
                    try:
                        fields = _decode_packed_message(msg, args.pico_topic)
                        if fields is None:
                            raise ValueError("unexpected PICO topic")
                        button_pub.publish(fields)
                        chunk = _parse_incoming_chunk(
                            fields, args.wrist_source
                        )
                    except Exception as exc:
                        skipped += 1
                        print(
                            f"[pico->smpl_ref] skipped invalid PICO pose: {exc}",
                            flush=True,
                        )
                        continue

                    newest_frame = int(chunk.frame_indices[-1])
                    progress = _classify_frame_progress(
                        newest_frame, last_valid_newest_frame
                    )
                    if progress == "duplicate":
                        duplicate_chunks += 1
                        continue
                    if progress == "restart":
                        previous_epoch = source_stream_epoch
                        source_stream_epoch = _new_stream_epoch(
                            source_stream_epoch
                        )
                        counter_restarts += 1
                        print(
                            "[pico->smpl_ref] PICO frame counter restarted; "
                            f"newest={newest_frame} "
                            f"previous={last_valid_newest_frame} "
                            f"source_epoch={previous_epoch}->{source_stream_epoch}",
                            flush=True,
                        )

                    source_fields = _source_chunk_fields(
                        chunk,
                        source_stream_epoch=source_stream_epoch,
                        received_monotonic_ns=int(received_mono * 1.0e9),
                    )
                    try:
                        pub.send(
                            pack_pose_message(
                                source_fields,
                                topic=args.out_topic,
                                version=5,
                            ),
                            flags=zmq.NOBLOCK,
                        )
                        sent += 1
                    except zmq.Again:
                        # Every source packet is a complete rolling chunk; if a
                        # slow subscriber fills the queue, the next packet can
                        # reconstruct the current stream without a playback ACK.
                        send_dropped += 1

                    last_valid_newest_frame = newest_frame
                    last_valid_input_mono = received_mono
                    stale_was_reported = False

            mono_now = time.monotonic()
            input_age = (
                mono_now - last_valid_input_mono
                if last_valid_input_mono is not None
                else float("inf")
            )
            if (
                last_valid_input_mono is not None
                and input_age > args.stale_warning_seconds
                and not stale_was_reported
            ):
                print(
                    "[pico->smpl_ref] WARN PICO source stale; "
                    f"age_ms={input_age * 1000.0:.0f}. "
                    "No synthetic window is published; policy consumes its "
                    "buffer then holds the last complete window.",
                    flush=True,
                )
                stale_was_reported = True

            if mono_now - last_log >= args.log_every:
                elapsed = max(1.0e-9, mono_now - started_mono)
                if last_valid_newest_frame is None:
                    print(
                        "[pico->smpl_ref] waiting for 10 consecutive PICO frames "
                        f"received={received} skipped={skipped}",
                        flush=True,
                    )
                else:
                    print(
                        "[pico->smpl_ref] source "
                        f"sent={sent} received={received} skipped={skipped} "
                        f"duplicates={duplicate_chunks} "
                        f"restarts={counter_restarts} dropped={send_dropped} "
                        f"newest={last_valid_newest_frame} "
                        f"source_epoch={source_stream_epoch} "
                        f"input_age_ms={input_age * 1000.0:.0f} "
                        f"forward_hz={sent / elapsed:.3f}",
                        flush=True,
                    )
                last_log = mono_now
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        cleanup_steps = (
            ("ROS button publisher", button_pub.close),
            ("PICO subscriber", lambda: sub.close(linger=0)),
            ("smpl_ref publisher", lambda: pub.close(linger=0)),
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
