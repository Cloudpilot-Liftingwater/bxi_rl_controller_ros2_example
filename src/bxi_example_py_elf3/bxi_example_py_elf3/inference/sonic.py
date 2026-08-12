"""SONIC teleoperation policy wrapper for the official ELF3 BXI controller.

This module is intentionally a policy/inference module only.  It does not
publish ActuatorCmds, call reset services, or own the robot state machine.  The
BXI demo remains the only motor-command publisher and calls this class from a
RobotControlState.
"""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional
from pathlib import Path

import numpy as np
import onnxruntime as ort
import zmq

from bxi_example_py_elf3.sonic_pico.streamed_smpl_ref import (
    IncomingChunk,
    StreamedSmplRefMerger,
)

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - allows source-tree tooling without ROS setup
    get_package_share_directory = None


HEADER_SIZE = 1280
WINDOW = 10
NUM_JOINTS = 29
SMPL_TOKENIZER_DIM = 840
PROPRIOCEPTION_DIM = 930
MODEL_INPUT_DIM = SMPL_TOKENIZER_DIM + PROPRIOCEPTION_DIM
ACTION_CLIP = 20.0

SMPL_JOINTS_START = 0
SMPL_ROOT_ORI_START = 720
SMPL_WRIST_START = 780

DTYPE_MAP = {
    "f32": np.dtype("<f4"),
    "f64": np.dtype("<f8"),
    "i32": np.dtype("<i4"),
    "i64": np.dtype("<i8"),
    "u8": np.dtype("u1"),
    "bool": np.dtype("?"),
}

def _find_package_data_file(relative_path: str) -> str:
    if get_package_share_directory is not None:
        try:
            package_data = (
                Path(get_package_share_directory("bxi_example_py_elf3"))
                / "data"
                / relative_path
            )
            if package_data.exists():
                return str(package_data)
        except Exception:
            pass

    source_file = Path(__file__).resolve()
    for parent in source_file.parents:
        source_data = parent / "data" / relative_path
        if source_data.exists():
            return str(source_data)

    return str(Path.cwd() / "src" / "bxi_example_py_elf3" / "data" / relative_path)


DEFAULT_MODEL_ONNX = _find_package_data_file(
    "sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx"
)
DEFAULT_STREAM_REFERENCE = _find_package_data_file(
    "sonic_reference/elf3_pico_stand_clean_001/stream_reference.npz"
)
DEFAULT_STAND_REFERENCE = _find_package_data_file(
    "sonic_reference/elf3_pico_stand_clean_001/stream_reference.npz"
)

JOINT_NAMES = (
    "waist_y_joint",
    "waist_x_joint",
    "waist_z_joint",
    "l_hip_y_joint",
    "l_hip_x_joint",
    "l_hip_z_joint",
    "l_knee_y_joint",
    "l_ankle_y_joint",
    "l_ankle_x_joint",
    "r_hip_y_joint",
    "r_hip_x_joint",
    "r_hip_z_joint",
    "r_knee_y_joint",
    "r_ankle_y_joint",
    "r_ankle_x_joint",
    "l_shoulder_y_joint",
    "l_shoulder_x_joint",
    "l_shoulder_z_joint",
    "l_elbow_y_joint",
    "l_wrist_x_joint",
    "l_wrist_y_joint",
    "l_wrist_z_joint",
    "r_shoulder_y_joint",
    "r_shoulder_x_joint",
    "r_shoulder_z_joint",
    "r_elbow_y_joint",
    "r_wrist_x_joint",
    "r_wrist_y_joint",
    "r_wrist_z_joint",
)

DEFAULT_DOF_POS = np.array(
    [
        0.0,
        0.0,
        0.0,
        -0.3,
        0.0,
        0.0,
        0.6,
        -0.3,
        0.0,
        -0.3,
        0.0,
        0.0,
        0.6,
        -0.3,
        0.0,
        0.2,
        0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)

KPS = np.array(
    [
        108.448,
        162.672,
        176.421,
        176.421,
        176.421,
        54.224,
        176.421,
        33.493,
        21.771,
        176.421,
        176.421,
        54.224,
        176.421,
        33.493,
        21.771,
        54.224,
        54.224,
        16.747,
        54.224,
        16.747,
        16.747,
        16.747,
        54.224,
        54.224,
        16.747,
        54.224,
        16.747,
        16.747,
        16.747,
    ],
    dtype=np.float32,
)

KDS = np.array(
    [
        6.904,
        10.356,
        11.231,
        11.231,
        11.231,
        3.452,
        11.231,
        2.132,
        1.386,
        11.231,
        11.231,
        3.452,
        11.231,
        2.132,
        1.386,
        3.452,
        3.452,
        1.066,
        3.452,
        1.066,
        1.066,
        1.066,
        3.452,
        3.452,
        1.066,
        3.452,
        1.066,
        1.066,
        1.066,
    ],
    dtype=np.float32,
)

ACTION_SCALE = np.array(
    [
        0.230525229,
        0.153683486,
        0.141706486,
        0.141706486,
        0.141706486,
        0.230525229,
        0.212559729,
        0.373212313,
        0.229663314,
        0.141706486,
        0.141706486,
        0.230525229,
        0.212559729,
        0.373212313,
        0.229663314,
        0.230525229,
        0.230525229,
        0.37320117,
        0.230525229,
        0.37320117,
        0.37320117,
        0.37320117,
        0.230525229,
        0.230525229,
        0.37320117,
        0.230525229,
        0.37320117,
        0.37320117,
        0.37320117,
    ],
    dtype=np.float32,
)


@dataclass
class SmplReferenceFrame:
    term1_local: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray
    anchor_quat: Optional[np.ndarray] = None
    frame_index: int = -1
    sequence: int = 0
    stream_epoch: Optional[int] = None
    source_stale: bool = False
    source_age_ms: Optional[float] = None
    playback_hold: bool = False
    newest_frame_index: int = -1
    lead_frames: int = -1
    valid_horizon: int = 0
    clamp_slots: int = -1


@dataclass(frozen=True, slots=True)
class PolicyPlaybackTelemetry:
    """Read-only observation of one successfully consumed live reference window."""

    frame_index: int
    newest_frame_index: int
    lead_frames: int
    playback_hold: bool
    catchup_count: int
    stream_epoch: Optional[int]
    successful_inference_tick: int


def _as_bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _normalize_quat_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm <= 1.0e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def _quat_mul_wxyz(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = lhs
    w2, x2, y2, z2 = rhs
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_conjugate_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _axis_angle_quat_wxyz(axis: str, angle: float) -> np.ndarray:
    half = 0.5 * float(angle)
    c = math.cos(half)
    s = math.sin(half)
    if axis == "x":
        return np.array([c, s, 0.0, 0.0], dtype=np.float64)
    if axis == "y":
        return np.array([c, 0.0, s, 0.0], dtype=np.float64)
    if axis == "z":
        return np.array([c, 0.0, 0.0, s], dtype=np.float64)
    raise ValueError(f"unsupported axis {axis}")


def _waist_z_quat_from_torso_wxyz(
    torso_quat_wxyz: np.ndarray,
    waist_y: float,
    waist_x: float,
    waist_z: float,
) -> np.ndarray:
    q = _normalize_quat_wxyz(torso_quat_wxyz)
    q = _quat_mul_wxyz(q, _axis_angle_quat_wxyz("y", waist_y))
    q = _quat_mul_wxyz(q, _axis_angle_quat_wxyz("x", waist_x))
    q = _quat_mul_wxyz(q, _axis_angle_quat_wxyz("z", waist_z))
    return _normalize_quat_wxyz(q)


def _projected_gravity_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize_quat_wxyz(q)
    v = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    qc = np.array([w, -x, -y, -z], dtype=np.float64)
    return np.array(
        [
            v[0] * (qc[0] * qc[0] + qc[1] * qc[1] - qc[2] * qc[2] - qc[3] * qc[3])
            + v[1] * 2.0 * (qc[1] * qc[2] - qc[0] * qc[3])
            + v[2] * 2.0 * (qc[1] * qc[3] + qc[0] * qc[2]),
            v[0] * 2.0 * (qc[1] * qc[2] + qc[0] * qc[3])
            + v[1] * (qc[0] * qc[0] - qc[1] * qc[1] + qc[2] * qc[2] - qc[3] * qc[3])
            + v[2] * 2.0 * (qc[2] * qc[3] - qc[0] * qc[1]),
            v[0] * 2.0 * (qc[1] * qc[3] - qc[0] * qc[2])
            + v[1] * 2.0 * (qc[2] * qc[3] + qc[0] * qc[1])
            + v[2] * (qc[0] * qc[0] - qc[1] * qc[1] - qc[2] * qc[2] + qc[3] * qc[3]),
        ],
        dtype=np.float32,
    )


def _sixd_from_quat_wxyz(q: np.ndarray) -> np.ndarray:
    r, i, j, k = _normalize_quat_wxyz(q)
    two_s = 2.0 / (r * r + i * i + j * j + k * k)
    return np.array(
        [
            1.0 - two_s * (j * j + k * k),
            two_s * (i * j - k * r),
            two_s * (i * j + k * r),
            1.0 - two_s * (i * i + k * k),
            two_s * (i * k - j * r),
            two_s * (j * k + i * r),
        ],
        dtype=np.float32,
    )


def _yaw_from_quat_wxyz(q: np.ndarray) -> float:
    w, x, y, z = _normalize_quat_wxyz(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _decode_packed_message(msg: bytes, topic: str) -> Optional[dict[str, np.ndarray]]:
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


def _as_window(arr: np.ndarray, width: int, name: str) -> np.ndarray:
    arr = np.asarray(arr, dtype=np.float32)
    if arr.ndim == 1:
        arr = arr.reshape(1, width)
    elif arr.ndim > 2:
        arr = arr.reshape(arr.shape[0], -1)
    if arr.shape[1] != width:
        raise ValueError(f"{name} has shape {arr.shape}; expected (*,{width})")
    if arr.shape[0] >= WINDOW:
        return np.ascontiguousarray(arr[:WINDOW], dtype=np.float32)
    return np.ascontiguousarray(
        np.concatenate([arr, np.repeat(arr[-1:], WINDOW - arr.shape[0], axis=0)]),
        dtype=np.float32,
    )


def _as_exact_live_window(arr: np.ndarray, width: int, name: str) -> np.ndarray:
    """Validate the no-clamp window contract used by the live v4 stream."""
    arr = np.asarray(arr, dtype=np.float32)
    if arr.ndim == 1:
        if arr.size != width:
            raise ValueError(
                f"{name} has shape {arr.shape}; expected ({WINDOW},{width})"
            )
        arr = arr.reshape(1, width)
    elif arr.ndim > 2:
        arr = arr.reshape(arr.shape[0], -1)
    if arr.ndim != 2 or arr.shape != (WINDOW, width):
        raise ValueError(
            f"{name} has shape {arr.shape}; expected ({WINDOW},{width})"
        )
    return np.ascontiguousarray(arr, dtype=np.float32)


STRICT_LIVE_WINDOW_METADATA = frozenset(
    (
        "stream_epoch",
        "valid_horizon",
        "clamp_slots",
    )
)


class SonicTeleopPolicy:
    """SONIC _smpl.onnx policy for BXI RobotControlState integration."""

    def __init__(
        self,
        model_onnx_path: Optional[str] = None,
        stream_reference_npz: Optional[str] = None,
        stand_reference_npz: Optional[str] = None,
        use_smpl_ref_zmq: Optional[bool] = None,
        smpl_ref_zmq_host: Optional[str] = None,
        smpl_ref_zmq_port: Optional[int] = None,
        smpl_ref_zmq_topic: Optional[str] = None,
        require_live_reference: Optional[bool] = None,
        yaw_bias_rad: Optional[float] = None,
    ):
        self.model_onnx_path = model_onnx_path or os.environ.get(
            "BXI_SONIC_MODEL_ONNX", DEFAULT_MODEL_ONNX
        )
        self.stream_reference_npz = stream_reference_npz or os.environ.get(
            "BXI_SONIC_STREAM_REFERENCE_NPZ", DEFAULT_STREAM_REFERENCE
        )
        self.stand_reference_npz = stand_reference_npz or os.environ.get(
            "BXI_SONIC_STAND_REFERENCE_NPZ", DEFAULT_STAND_REFERENCE
        )
        self.use_smpl_ref_zmq = (
            _as_bool_env("BXI_SONIC_USE_SMPL_REF_ZMQ", True)
            if use_smpl_ref_zmq is None
            else bool(use_smpl_ref_zmq)
        )
        self.smpl_ref_zmq_host = smpl_ref_zmq_host or os.environ.get(
            "BXI_SONIC_SMPL_REF_ZMQ_HOST", "127.0.0.1"
        )
        self.smpl_ref_zmq_port = int(
            smpl_ref_zmq_port
            if smpl_ref_zmq_port is not None
            else os.environ.get("BXI_SONIC_SMPL_REF_ZMQ_PORT", "5557")
        )
        self.smpl_ref_zmq_topic = smpl_ref_zmq_topic or os.environ.get(
            "BXI_SONIC_SMPL_REF_ZMQ_TOPIC", "smpl_ref"
        )
        self.require_live_reference = (
            _as_bool_env("BXI_SONIC_REQUIRE_LIVE_REFERENCE", True)
            if require_live_reference is None
            else bool(require_live_reference)
        )
        self.yaw_bias_rad = float(
            yaw_bias_rad
            if yaw_bias_rad is not None
            else os.environ.get("BXI_SONIC_YAW_BIAS_RAD", "1.57079632679")
        )
        self.live_ref_timeout_s = float(os.environ.get("BXI_SONIC_LIVE_REF_TIMEOUT_S", "0.5"))

        self.default_dof_pos = DEFAULT_DOF_POS.copy()
        self.target_dof_pos = DEFAULT_DOF_POS.copy()
        self.kps = KPS.copy()
        self.kds = KDS.copy()
        self.action_scale = ACTION_SCALE.copy()
        self.joint_names = JOINT_NAMES
        self.obs_history_len = WINDOW

        self.last_action = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.base_ang_vel_history = np.zeros((WINDOW, 3), dtype=np.float32)
        self.joint_pos_history = np.zeros((WINDOW, NUM_JOINTS), dtype=np.float32)
        self.joint_vel_history = np.zeros((WINDOW, NUM_JOINTS), dtype=np.float32)
        self.action_history = np.zeros((WINDOW, NUM_JOINTS), dtype=np.float32)
        self.gravity_history = np.zeros((WINDOW, 3), dtype=np.float32)

        self.motion_cursor = 0
        self.yaw_aligned = False
        self.yaw_offset = 0.0
        self.stream_merger = StreamedSmplRefMerger()
        self.source_stream_epoch: Optional[int] = None
        self.last_source_newest_frame: Optional[int] = None
        self.last_source_rx_mono = 0.0
        self.source_chunk_messages = 0
        self.source_chunk_duplicates = 0
        self.source_chunk_restarts = 0
        self.source_queue_drops = 0
        self.has_seen_live_reference = False
        self.live_reference_protocol = "none"
        self.active_reference_kind = "none"
        self.latest_live_ref: Optional[SmplReferenceFrame] = None
        self.latest_live_ref_time = 0.0
        self.live_sequence = 0
        self.stream_epoch: Optional[int] = None
        self.invalid_live_ref_messages = 0
        self.live_reference_stale = False
        self.policy_active = False
        self.last_status = "not_started"
        self.successful_inference_tick = 0
        self.latest_playback_telemetry: Optional[PolicyPlaybackTelemetry] = None
        self.telemetry_log_every = max(
            0, int(os.environ.get("BXI_SONIC_TELEMETRY_LOG_EVERY", "0"))
        )

        self._load_stream_reference()
        self._init_onnx()
        self._init_zmq()

    def _init_onnx(self) -> None:
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if ort.get_device() == "GPU" else ["CPUExecutionProvider"]
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            self.model_onnx_path,
            providers=providers,
            sess_options=options,
        )
        self.input_info = self.session.get_inputs()[0]
        self.output_info = self.session.get_outputs()[0]
        self.input_buffer = np.zeros((1, MODEL_INPUT_DIM), dtype=np.float32)
        if int(self.input_info.shape[-1]) != MODEL_INPUT_DIM:
            raise ValueError(
                f"SONIC ONNX input dim is {self.input_info.shape}; expected (1,{MODEL_INPUT_DIM})"
            )

    def _init_zmq(self) -> None:
        # pyzmq sockets are not thread-safe.  One dedicated I/O thread owns the
        # SUB socket; the ROS/control thread alone owns merger and playback state.
        self.zmq_context = None
        self.zmq_socket = None
        self.zmq_poller = None
        self._zmq_inbound_queue: queue.Queue[
            tuple[bytes, float]
        ] = queue.Queue(maxsize=64)
        self._zmq_stop_event = threading.Event()
        self._zmq_ready_event = threading.Event()
        self._zmq_thread: Optional[threading.Thread] = None
        self._zmq_io_thread_id: Optional[int] = None
        self._zmq_start_error: Optional[str] = None
        if not self.use_smpl_ref_zmq:
            self._zmq_ready_event.set()
            return

        self._zmq_thread = threading.Thread(
            target=self._zmq_io_loop,
            name="sonic-smpl-ref-zmq",
            daemon=True,
        )
        self._zmq_thread.start()
        self._zmq_ready_event.wait(timeout=1.0)

    def _queue_reference_message(
        self,
        message: bytes,
        received_mono: float,
    ) -> None:
        item = (message, float(received_mono))
        try:
            self._zmq_inbound_queue.put_nowait(item)
            return
        except queue.Full:
            pass

        # Source packets are complete rolling chunks.  Preserve ordering until
        # the bounded queue really overflows, then discard only the oldest one.
        try:
            self._zmq_inbound_queue.get_nowait()
            self.source_queue_drops += 1
        except queue.Empty:
            pass
        try:
            self._zmq_inbound_queue.put_nowait(item)
        except queue.Full:
            self.source_queue_drops += 1

    def _zmq_io_loop(self) -> None:
        context = None
        ref_socket = None
        poller = None
        self._zmq_io_thread_id = threading.get_ident()
        try:
            context = zmq.Context()
            ref_socket = context.socket(zmq.SUB)
            ref_socket.setsockopt(zmq.LINGER, 0)
            ref_socket.setsockopt(zmq.RCVHWM, 64)
            ref_socket.setsockopt_string(
                zmq.SUBSCRIBE, self.smpl_ref_zmq_topic
            )
            ref_socket.connect(
                f"tcp://{self.smpl_ref_zmq_host}:{self.smpl_ref_zmq_port}"
            )
            poller = zmq.Poller()
            poller.register(ref_socket, zmq.POLLIN)
            self._zmq_ready_event.set()

            while not self._zmq_stop_event.is_set():
                events = dict(poller.poll(timeout=10))
                if ref_socket not in events:
                    continue
                while True:
                    try:
                        message = ref_socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    self._queue_reference_message(
                        message, time.monotonic()
                    )
        except Exception as exc:
            self._zmq_start_error = repr(exc)
            self._zmq_ready_event.set()
        finally:
            if poller is not None and ref_socket is not None:
                try:
                    poller.unregister(ref_socket)
                except Exception:
                    pass
            if ref_socket is not None:
                try:
                    ref_socket.close(linger=0)
                except Exception:
                    pass
            if context is not None:
                try:
                    context.term()
                except Exception:
                    pass
            self._zmq_ready_event.set()
    def close(self) -> None:
        """Signal and join the sole owner of all policy ZMQ resources."""
        stop_event = getattr(self, "_zmq_stop_event", None)
        if stop_event is not None:
            stop_event.set()
        thread = getattr(self, "_zmq_thread", None)
        if (
            thread is not None
            and thread.is_alive()
            and thread.ident != threading.get_ident()
        ):
            thread.join(timeout=2.0)
        if thread is not None and not thread.is_alive():
            self._zmq_thread = None

    def __del__(self) -> None:  # pragma: no cover - process-shutdown fallback
        try:
            self.close()
        except Exception:
            pass

    @staticmethod
    def _load_reference_npz(
        path: str,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, Optional[np.ndarray]]:
        with np.load(path) as data:
            term1 = np.asarray(data["term1_local"], dtype=np.float32)
            root = np.asarray(data["root_quat"], dtype=np.float32)
            wrist = np.asarray(data["wrist"], dtype=np.float32)
            anchor = (
                np.asarray(data["anchor_quat"], dtype=np.float32)
                if "anchor_quat" in data.files
                else None
            )
        if term1.ndim != 2 or term1.shape[1] != 72:
            raise ValueError(
                f"{path}: term1_local shape {term1.shape}, expected (T,72)"
            )
        if root.shape != (term1.shape[0], 4):
            raise ValueError(f"{path}: root_quat shape does not match term1_local")
        if wrist.shape != (term1.shape[0], 6):
            raise ValueError(f"{path}: wrist shape does not match term1_local")
        if anchor is not None and anchor.shape != (term1.shape[0], 4):
            raise ValueError(f"{path}: anchor_quat shape does not match term1_local")
        return term1, root, wrist, anchor

    def _load_stream_reference(self) -> None:
        (
            self.ref_term1,
            self.ref_root_quat,
            self.ref_wrist,
            self.ref_anchor_quat,
        ) = self._load_reference_npz(self.stream_reference_npz)
        (
            self.stand_term1,
            self.stand_root_quat,
            self.stand_wrist,
            self.stand_anchor_quat,
        ) = self._load_reference_npz(self.stand_reference_npz)
        if self.stand_term1.shape[0] < WINDOW:
            raise ValueError(
                f"{self.stand_reference_npz}: need at least {WINDOW} stand frames"
            )

    def reset(self) -> None:
        self.last_action.fill(0.0)
        self.base_ang_vel_history.fill(0.0)
        self.joint_pos_history.fill(0.0)
        self.joint_vel_history.fill(0.0)
        self.action_history.fill(0.0)
        self.gravity_history.fill(0.0)
        self.motion_cursor = 0
        self.reset_yaw_alignment()
        self.stream_merger.reset()
        self.source_stream_epoch = None
        self.last_source_newest_frame = None
        self.last_source_rx_mono = 0.0
        self.has_seen_live_reference = False
        self.live_reference_protocol = "none"
        self.active_reference_kind = "none"
        self.latest_live_ref = None
        self.latest_live_ref_time = 0.0
        self.live_sequence = 0
        self.stream_epoch = None
        self.live_reference_stale = False
        self.policy_active = False
        self.last_status = "reset"
        self.latest_playback_telemetry = None
        self.target_dof_pos = self.default_dof_pos.copy()

        inbound = getattr(self, "_zmq_inbound_queue", None)
        if inbound is not None:
            while True:
                try:
                    inbound.get_nowait()
                except queue.Empty:
                    break

    def reset_yaw_alignment(self) -> None:
        self.yaw_aligned = False
        self.yaw_offset = 0.0

    @staticmethod
    def _source_chunk_from_fields(
        fields: dict[str, np.ndarray],
    ) -> IncomingChunk:
        frame_indices = np.asarray(
            fields["frame_index"], dtype=np.int64
        ).reshape(-1)
        n = int(frame_indices.size)
        if n < WINDOW:
            raise ValueError(
                f"source chunk has {n} frames; need at least {WINDOW}"
            )
        if np.any(np.diff(frame_indices) != 1):
            raise ValueError(
                "source chunk frame_index must be consecutive: "
                f"{frame_indices.tolist()}"
            )

        def matrix(name: str, width: int) -> np.ndarray:
            arr = np.asarray(fields[name], dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(1, width)
            elif arr.ndim > 2:
                arr = arr.reshape(arr.shape[0], -1)
            if arr.shape != (n, width):
                raise ValueError(
                    f"source chunk {name} has shape {arr.shape}; "
                    f"expected ({n},{width})"
                )
            return np.ascontiguousarray(arr, dtype=np.float32)

        return IncomingChunk(
            frame_indices=np.ascontiguousarray(
                frame_indices, dtype=np.int64
            ),
            term1_local=matrix("term1_local", 72),
            root_quat=matrix("root_quat", 4),
            wrist=matrix("wrist", 6),
        )

    @staticmethod
    def _field_scalar(
        fields: dict[str, np.ndarray],
        name: str,
        default: Any = None,
    ) -> Any:
        value = fields.get(name)
        if value is None or np.asarray(value).size == 0:
            return default
        return np.asarray(value).reshape(-1)[-1]

    def _merge_source_fields(
        self,
        fields: dict[str, np.ndarray],
        received_mono: float,
    ) -> bool:
        chunk = self._source_chunk_from_fields(fields)
        source_epoch = int(
            self._field_scalar(fields, "source_stream_epoch", 0)
        )
        if source_epoch <= 0:
            raise ValueError("source chunk missing positive source_stream_epoch")

        if self.source_stream_epoch != source_epoch:
            self.stream_merger.reset()
            self.source_stream_epoch = source_epoch
            self.last_source_newest_frame = None
            self.source_chunk_restarts += int(
                self.has_seen_live_reference
            )

        newest = int(chunk.frame_indices[-1])
        if self.last_source_newest_frame is not None:
            if newest == self.last_source_newest_frame:
                self.source_chunk_duplicates += 1
                return False
            if newest < self.last_source_newest_frame:
                # A same-epoch counter rollback is still treated atomically as
                # a new stream; normal bridge restarts carry a new source epoch.
                self.stream_merger.reset()
                self.last_source_newest_frame = None
                self.source_chunk_restarts += 1

        self.stream_merger.merge(chunk)
        self.last_source_newest_frame = newest
        self.last_source_rx_mono = float(received_mono)
        self.source_chunk_messages += 1
        return True

    def poll_reference(self) -> Optional[SmplReferenceFrame]:
        inbound = getattr(self, "_zmq_inbound_queue", None)
        if inbound is not None:
            while True:
                try:
                    queued = inbound.get_nowait()
                except queue.Empty:
                    break
                if isinstance(queued, tuple):
                    msg, received_mono = queued
                else:  # compatibility for older tests/tools
                    msg, received_mono = queued, time.monotonic()
                try:
                    fields = _decode_packed_message(
                        msg, self.smpl_ref_zmq_topic
                    )
                    if not fields:
                        raise ValueError("invalid smpl_ref message")
                    if bool(
                        self._field_scalar(fields, "source_chunk", False)
                    ):
                        self._merge_source_fields(
                            fields, float(received_mono)
                        )
                        continue

                    frame = self._frame_from_fields(fields)
                except Exception:
                    self.invalid_live_ref_messages += 1
                    continue

                if not self.has_seen_live_reference:
                    self.reset_yaw_alignment()
                self.has_seen_live_reference = True
                # An explicit legacy-window publisher/replay replaces the
                # source-chunk protocol; do not let an old merger buffer win
                # again on the next control tick.
                self.stream_merger.reset()
                self.source_stream_epoch = None
                self.last_source_newest_frame = None
                self.live_reference_protocol = "legacy_window"
                self.latest_live_ref = frame
                self.latest_live_ref_time = float(received_mono)

        if self.stream_merger.timesteps >= WINDOW:
            now = time.monotonic()
            age_s = max(0.0, now - self.last_source_rx_mono)
            fields = self.stream_merger.build_smpl_ref(
                source_age_ms=age_s * 1000.0,
                source_stale=age_s > self.live_ref_timeout_s,
            )
            if fields is not None:
                if not self.has_seen_live_reference:
                    self.reset_yaw_alignment()
                self.has_seen_live_reference = True
                self.live_reference_protocol = "source_chunk"
                self.latest_live_ref = self._frame_from_fields(fields)
                self.latest_live_ref_time = self.last_source_rx_mono

        return self.latest_live_ref

    def _frame_from_fields(self, fields: dict[str, np.ndarray]) -> SmplReferenceFrame:
        self.live_sequence += 1

        def scalar(name: str, default: Any) -> Any:
            value = fields.get(name)
            if value is None or np.asarray(value).size == 0:
                return default
            return np.asarray(value).reshape(-1)[-1]

        source_age_ms = scalar("source_age_ms", None)
        valid_horizon = scalar("valid_horizon", 0)
        clamp_slots = scalar("clamp_slots", -1)
        strict_live_window = any(
            name in fields for name in STRICT_LIVE_WINDOW_METADATA
        )
        if strict_live_window:
            if int(valid_horizon) != WINDOW:
                raise ValueError(
                    "live v4 reference must declare "
                    f"valid_horizon={WINDOW}; got {int(valid_horizon)}"
                )
            if int(clamp_slots) != 0:
                raise ValueError(
                    "live v4 reference must declare clamp_slots=0; "
                    f"got {int(clamp_slots)}"
                )
            as_live_window = _as_exact_live_window
        else:
            # Preserve the pre-v4 live publisher contract.  Offline playback
            # has its own clamped cursor path in _offline_frame().
            as_live_window = _as_window
        anchor = fields.get("anchor_quat")
        return SmplReferenceFrame(
            term1_local=as_live_window(fields["term1_local"], 72, "term1_local"),
            root_quat=as_live_window(fields["root_quat"], 4, "root_quat"),
            wrist=as_live_window(fields["wrist"], 6, "wrist"),
            anchor_quat=(
                as_live_window(anchor, 4, "anchor_quat")
                if anchor is not None
                else None
            ),
            frame_index=int(scalar("frame_index", -1)),
            sequence=self.live_sequence,
            stream_epoch=(
                int(scalar("stream_epoch", 0))
                if fields.get("stream_epoch") is not None
                else None
            ),
            source_stale=bool(scalar("source_stale", False)),
            source_age_ms=(float(source_age_ms) if source_age_ms is not None else None),
            playback_hold=bool(scalar("playback_hold", False)),
            newest_frame_index=int(scalar("newest_frame_index", -1)),
            lead_frames=int(scalar("lead_frames", -1)),
            valid_horizon=int(valid_horizon),
            clamp_slots=int(clamp_slots),
        )

    def _stand_frame(self) -> SmplReferenceFrame:
        idx = np.arange(WINDOW, dtype=np.int64)
        anchor = (
            self.stand_anchor_quat[idx]
            if self.stand_anchor_quat is not None
            else None
        )
        return SmplReferenceFrame(
            term1_local=np.ascontiguousarray(self.stand_term1[idx]),
            root_quat=np.ascontiguousarray(self.stand_root_quat[idx]),
            wrist=np.ascontiguousarray(self.stand_wrist[idx]),
            anchor_quat=(
                np.ascontiguousarray(anchor) if anchor is not None else None
            ),
            frame_index=0,
            sequence=0,
        )

    def _offline_frame(self) -> SmplReferenceFrame:
        t = self.ref_term1.shape[0]
        idx = np.minimum(np.arange(self.motion_cursor, self.motion_cursor + WINDOW), t - 1)
        anchor = self.ref_anchor_quat[idx] if self.ref_anchor_quat is not None else None
        return SmplReferenceFrame(
            term1_local=np.ascontiguousarray(self.ref_term1[idx], dtype=np.float32),
            root_quat=np.ascontiguousarray(self.ref_root_quat[idx], dtype=np.float32),
            wrist=np.ascontiguousarray(self.ref_wrist[idx], dtype=np.float32),
            anchor_quat=np.ascontiguousarray(anchor, dtype=np.float32) if anchor is not None else None,
            frame_index=int(idx[0]),
            sequence=0,
        )

    def _active_reference(self) -> Optional[SmplReferenceFrame]:
        live = self.poll_reference()
        if live is not None:
            if live.stream_epoch is not None and live.stream_epoch != self.stream_epoch:
                self.stream_epoch = live.stream_epoch
                # Match the official stream-restart semantics: the reference
                # heading belongs to the new stream epoch, while real robot
                # proprioception/action history remains continuous.
                self.reset_yaw_alignment()

            local_age_s = max(0.0, time.monotonic() - self.latest_live_ref_time)
            source_age_stale = (
                live.source_age_ms is not None
                and live.source_age_ms > self.live_ref_timeout_s * 1000.0
            )
            self.live_reference_stale = bool(
                live.source_stale
                or source_age_stale
                or local_age_s > self.live_ref_timeout_s
            )
            self.active_reference_kind = self.live_reference_protocol
            # Once a complete live reference has arrived, retain it without a
            # timeout.  This mirrors the official policy-side boundary hold:
            # source chunks are consumed to the protected tail, then the last
            # complete window remains active indefinitely.
            return live

        self.live_reference_stale = False
        if self.require_live_reference:
            self.active_reference_kind = "standby"
            return self._stand_frame()

        self.active_reference_kind = "offline"
        return self._offline_frame()

    def _update_history(self, q: np.ndarray, dq: np.ndarray, quat_wxyz: np.ndarray, omega: np.ndarray) -> np.ndarray:
        anchor = _waist_z_quat_from_torso_wxyz(quat_wxyz, q[0], q[1], q[2])
        gravity = _projected_gravity_from_quat_wxyz(anchor)
        self.base_ang_vel_history[:-1] = self.base_ang_vel_history[1:]
        self.joint_pos_history[:-1] = self.joint_pos_history[1:]
        self.joint_vel_history[:-1] = self.joint_vel_history[1:]
        self.action_history[:-1] = self.action_history[1:]
        self.gravity_history[:-1] = self.gravity_history[1:]
        self.base_ang_vel_history[-1] = np.asarray(omega, dtype=np.float32).reshape(3)
        self.joint_pos_history[-1] = np.asarray(q, dtype=np.float32).reshape(NUM_JOINTS) - self.default_dof_pos
        self.joint_vel_history[-1] = np.asarray(dq, dtype=np.float32).reshape(NUM_JOINTS)
        self.action_history[-1] = self.last_action
        self.gravity_history[-1] = gravity
        return anchor

    def _capture_yaw_if_needed(self, frame: SmplReferenceFrame, anchor_quat_wxyz: np.ndarray) -> None:
        if self.yaw_aligned:
            return
        if frame.anchor_quat is not None:
            reference_yaw = _yaw_from_quat_wxyz(frame.anchor_quat[0])
            bias = 0.0
        else:
            reference_yaw = _yaw_from_quat_wxyz(frame.root_quat[0])
            bias = self.yaw_bias_rad
        self.yaw_offset = reference_yaw - _yaw_from_quat_wxyz(anchor_quat_wxyz) + bias
        self.yaw_aligned = True

    def _write_smpl_tokenizer(
        self,
        frame: SmplReferenceFrame,
        anchor_quat_wxyz: np.ndarray,
        out: np.ndarray,
    ) -> None:
        out[SMPL_JOINTS_START:SMPL_JOINTS_START + 720] = frame.term1_local.reshape(-1)
        out[SMPL_WRIST_START:SMPL_WRIST_START + 60] = frame.wrist.reshape(-1)
        conj_anchor = _quat_conjugate_wxyz(anchor_quat_wxyz)
        for k in range(WINDOW):
            root_quat = _normalize_quat_wxyz(frame.root_quat[k])
            rel = _quat_mul_wxyz(conj_anchor, root_quat)
            out[SMPL_ROOT_ORI_START + k * 6:SMPL_ROOT_ORI_START + (k + 1) * 6] = _sixd_from_quat_wxyz(rel)

    def _build_model_input(
        self,
        frame: SmplReferenceFrame,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        omega: np.ndarray,
    ) -> np.ndarray:
        anchor = self._update_history(q, dq, quat_wxyz, omega)
        self._capture_yaw_if_needed(frame, anchor)
        anchor_aligned = _quat_mul_wxyz(_axis_angle_quat_wxyz("z", self.yaw_offset), anchor)

        model_input = np.zeros(MODEL_INPUT_DIM, dtype=np.float32)
        self._write_smpl_tokenizer(frame, anchor_aligned, model_input)
        proprio = np.concatenate(
            [
                self.base_ang_vel_history.reshape(-1),
                self.joint_pos_history.reshape(-1),
                self.joint_vel_history.reshape(-1),
                self.action_history.reshape(-1),
                self.gravity_history.reshape(-1),
            ]
        ).astype(np.float32)
        model_input[SMPL_TOKENIZER_DIM:] = proprio
        return model_input.reshape(1, -1)

    def _record_playback_telemetry(
        self,
        frame: SmplReferenceFrame,
        *,
        advanced: bool,
    ) -> None:
        """Publish a local snapshot after, and only after, a successful live tick."""
        self.successful_inference_tick += 1
        telemetry = PolicyPlaybackTelemetry(
            frame_index=int(frame.frame_index),
            newest_frame_index=int(frame.newest_frame_index),
            lead_frames=int(frame.lead_frames),
            playback_hold=not bool(advanced),
            catchup_count=int(self.stream_merger.catchup_count),
            stream_epoch=(
                int(frame.stream_epoch) if frame.stream_epoch is not None else None
            ),
            successful_inference_tick=self.successful_inference_tick,
        )
        self.latest_playback_telemetry = telemetry

        if (
            self.telemetry_log_every > 0
            and self.successful_inference_tick % self.telemetry_log_every == 0
        ):
            payload = {
                "frame_index": telemetry.frame_index,
                "newest_frame_index": telemetry.newest_frame_index,
                "lead_frames": telemetry.lead_frames,
                "playback_hold": telemetry.playback_hold,
                "catchup_count": telemetry.catchup_count,
                "stream_epoch": telemetry.stream_epoch,
                "successful_inference_tick": telemetry.successful_inference_tick,
            }
            print(
                "[sonic-playback-telemetry] "
                + json.dumps(payload, sort_keys=True, separators=(",", ":")),
                flush=True,
            )

    def _inference_step_impl(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        omega: np.ndarray,
        *,
        preheat: bool,
    ) -> np.ndarray:
        q = np.asarray(q, dtype=np.float32).reshape(NUM_JOINTS)
        dq = np.asarray(dq, dtype=np.float32).reshape(NUM_JOINTS)
        quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
        omega = np.asarray(omega, dtype=np.float32).reshape(3)

        if preheat:
            # Model/session warmup is executed in a tight loop before the
            # state starts producing motor commands.  Use the recorded stand
            # reference without polling the live queue, and never consume the
            # official playback cursor from a non-control tick.
            frame = self._stand_frame()
            self.active_reference_kind = "standby"
            self.live_reference_stale = False
        else:
            frame = self._active_reference()
        if frame is None:
            self._update_history(q, dq, quat_wxyz, omega)
            self.target_dof_pos = self.default_dof_pos.copy()
            self.policy_active = False
            self.last_status = "waiting_for_reference"
            return self.target_dof_pos

        model_input = self._build_model_input(frame, q, dq, quat_wxyz, omega)
        np.copyto(self.input_buffer, model_input)
        raw_action = self.session.run(
            [self.output_info.name], {self.input_info.name: self.input_buffer}
        )[0].reshape(-1)
        action = np.clip(raw_action, -ACTION_CLIP, ACTION_CLIP).astype(np.float32)
        self.last_action = action
        self.target_dof_pos = self.default_dof_pos + action * self.action_scale
        self.policy_active = True
        if self.active_reference_kind == "standby":
            self.last_status = "standby_reference"
        else:
            self.last_status = (
                "stale_hold" if self.live_reference_stale else "policy"
            )

        # Official order: gather -> successful inference/action -> advance.
        if not preheat:
            if self.active_reference_kind == "source_chunk":
                advanced = self.stream_merger.advance_after_successful_tick()
                self._record_playback_telemetry(frame, advanced=advanced)
            elif self.active_reference_kind == "offline":
                self.motion_cursor = min(
                    self.motion_cursor + 1,
                    self.ref_term1.shape[0] - 1,
                )
        return self.target_dof_pos

    def preheat_step(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        omega: np.ndarray,
    ) -> np.ndarray:
        """Warm ONNX/history on stand without touching the live time axis."""
        return self._inference_step_impl(
            q,
            dq,
            quat_wxyz,
            omega,
            preheat=True,
        )

    def inference_step(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        quat_wxyz: np.ndarray,
        omega: np.ndarray,
    ) -> np.ndarray:
        return self._inference_step_impl(
            q,
            dq,
            quat_wxyz,
            omega,
            preheat=False,
        )
