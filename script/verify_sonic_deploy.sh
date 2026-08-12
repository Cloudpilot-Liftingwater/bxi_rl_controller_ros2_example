#!/usr/bin/env bash
# Offline smoke checks for an installed SONIC ELF3 runtime.
set -Eeuo pipefail

RUNTIME=""
BXI_PREFIX=""
T3_WAIT_SMOKE=0

usage() {
  printf '%s\n' \
    "Usage: $0 --runtime DIR [--bxi-prefix DIR] [--t3-wait-smoke]"
}

while (($#)); do
  case "$1" in
    --runtime)
      RUNTIME="$2"
      shift 2
      ;;
    --bxi-prefix)
      BXI_PREFIX="$2"
      shift 2
      ;;
    --t3-wait-smoke)
      T3_WAIT_SMOKE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[verify] unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${RUNTIME}" && -d "${RUNTIME}" ]] || {
  printf '[verify] --runtime must name an installed release\n' >&2
  exit 3
}
[[ -f "${RUNTIME}/install/local_setup.bash" || -f "${RUNTIME}/install/setup.bash" ]] || {
  printf '[verify] runtime has not been built: %s/install\n' "${RUNTIME}" >&2
  exit 3
}
[[ -x "${RUNTIME}/.venv_teleop/bin/python" ]] || {
  printf '[verify] runtime Python environment is missing\n' >&2
  exit 3
}

if [[ -z "${BXI_PREFIX}" ]]; then
  for candidate in /opt/bxi/bxi_ros2_pkg /opt/bxi/bxi_ros2_pkg-main; do
    if [[ -f "${candidate}/local_setup.bash" || -f "${candidate}/setup.bash" ]]; then
      BXI_PREFIX="${candidate}"
      break
    fi
  done
fi
[[ -n "${BXI_PREFIX}" ]] || {
  printf '[verify] BXI ROS package prefix was not found\n' >&2
  exit 3
}

robot_states="${RUNTIME}/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py"
python3 - "${robot_states}" <<'PY'
import ast
import sys
from pathlib import Path

tree = ast.parse(Path(sys.argv[1]).read_text(encoding="utf-8"))
classes = {node.name: node for node in tree.body if isinstance(node, ast.ClassDef)}

def calls_orientation_gate(class_name: str) -> bool:
    cls = classes[class_name]
    method = next(
        node for node in cls.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "on_update"
    )
    return any(
        isinstance(node, ast.Attribute) and node.attr == "is_orientation_unsafe"
        for node in ast.walk(method)
    )

assert not calls_orientation_gate("SonicTeleopState")
assert calls_orientation_gate("NormalState")
print("[verify] orientation policy: sonic=bypass normal=protected")
PY

(
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "${BXI_PREFIX}/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${BXI_PREFIX}/local_setup.bash"
  else
    # shellcheck disable=SC1090
    source "${BXI_PREFIX}/setup.bash"
  fi
  if [[ -f "${RUNTIME}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${RUNTIME}/install/local_setup.bash"
  else
    # shellcheck disable=SC1090
    source "${RUNTIME}/install/setup.bash"
  fi
  set -u

  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/sonic_ros_logs_${UID}}"
  mkdir -p "${ROS_LOG_DIR}"
  venv_site="$("${RUNTIME}/.venv_teleop/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
  export PYTHONPATH="${venv_site}:${PYTHONPATH:-}"

  /usr/bin/python3 - "${RUNTIME}" <<'PY'
import sys
from pathlib import Path

import msgpack
import numpy
import onnx
import onnxruntime
import pinocchio
import scipy
import xrobotoolkit_sdk
import zmq
from ament_index_python.packages import get_package_share_directory
from bxi_example_py_elf3.sonic_pico.elf3_fk_calibration import Elf3FkCalibration

runtime = Path(sys.argv[1])
share = Path(get_package_share_directory("bxi_example_py_elf3"))
model = share / "data/sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx"
reference = share / "data/sonic_reference/elf3_pico_stand_clean_001/stream_reference.npz"
assert model.is_file(), model
assert reference.is_file(), reference

session = onnxruntime.InferenceSession(str(model), providers=["CPUExecutionProvider"])
inputs = session.get_inputs()
outputs = session.get_outputs()
assert any(item.shape and item.shape[-1] == 1770 for item in inputs), [item.shape for item in inputs]
assert any(item.shape and item.shape[-1] == 29 for item in outputs), [item.shape for item in outputs]

fk = Elf3FkCalibration.from_default_urdf()
poses = fk.key_frame_poses()
assert set(poses) == {"left_wrist", "right_wrist", "torso", "anchor"}

print("[verify] Python/ROS imports OK")
print(f"[verify] ONNX contract inputs={[i.shape for i in inputs]} outputs={[o.shape for o in outputs]}")
print(f"[verify] ELF3 FK frames={sorted(poses)}")
PY

  ros2 pkg prefix bxi_example_py_elf3 >/dev/null
  ros2 pkg prefix remote_controller >/dev/null
  ros2 launch bxi_example_py_elf3 example_sonic_sim2sim.launch.py --show-args >/dev/null
)

sdk_so="$(find "${RUNTIME}/.venv_teleop" -type f -name 'xrobotoolkit_sdk*.so' -print -quit)"
[[ -n "${sdk_so}" ]] || {
  printf '[verify] xrobotoolkit_sdk extension is missing\n' >&2
  exit 4
}
if ldd "${sdk_so}" | rg -q 'not found'; then
  ldd "${sdk_so}" >&2
  exit 4
fi
printf '[verify] XRT SDK linkage OK\n'

if ((T3_WAIT_SMOKE)); then
  printf '[verify] starting T3 wait-state shutdown smoke\n'
  t3_log="/tmp/sonic_t3_wait_smoke_$$.log"
  python3 - "${RUNTIME}/script/run_sonic_pico_sources.sh" "${t3_log}" <<'PY'
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

script = Path(sys.argv[1])
log_path = Path(sys.argv[2])
with log_path.open("wb") as log_file:
    process = subprocess.Popen(
        ["bash", str(script)],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(5.0)
    if process.poll() is not None:
        print("[verify] T3 exited before shutdown smoke", file=sys.stderr)
        sys.stderr.write(log_path.read_text(encoding="utf-8", errors="replace"))
        raise SystemExit(5)

    os.kill(process.pid, signal.SIGINT)
    try:
        return_code = process.wait(timeout=15.0)
    except subprocess.TimeoutExpired:
        os.kill(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.kill(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)
        print("[verify] T3 did not exit after SIGINT", file=sys.stderr)
        sys.stderr.write(log_path.read_text(encoding="utf-8", errors="replace"))
        raise SystemExit(5)

if return_code != 130:
    print(f"[verify] T3 shutdown returned {return_code}, expected 130", file=sys.stderr)
    sys.stderr.write(log_path.read_text(encoding="utf-8", errors="replace"))
    raise SystemExit(5)
PY
  if pgrep -af 'pico_manager_thread_server|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' >/dev/null; then
    printf '[verify] T3 child process remains after shutdown\n' >&2
    pgrep -af 'pico_manager_thread_server|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' >&2 || true
    exit 5
  fi
  for port in 5556 5557 60061; do
    if ss -ltn "sport = :${port}" | tail -n +2 | rg -q .; then
      printf '[verify] port %s remains in LISTEN after T3 shutdown\n' "${port}" >&2
      exit 5
    fi
  done
  rm -f "${t3_log}"
  printf '[verify] T3 wait-state shutdown OK\n'
fi

printf '[verify] all requested checks passed\n'
