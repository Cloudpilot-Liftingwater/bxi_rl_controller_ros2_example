#!/usr/bin/env bash
# Terminal 3: start the official PICO manager and the ELF3 smpl_ref bridge.
#
# Keep this terminal running while using the PICO.  Use the PICO visualizers to
# complete CALIB_FULL and switch into POSE.  Do not release MuJoCo manually:
# the deploy releases after the BXI gate is in sonic_teleop and the POSE policy
# has actually published targets.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"
source /opt/ros/humble/setup.bash
if [[ -f /opt/bxi/bxi_ros2_pkg-main/setup.bash ]]; then
  source /opt/bxi/bxi_ros2_pkg-main/setup.bash
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  source "${REPO_ROOT}/install/setup.bash"
fi
# Run the packaged bridge implementation, which owns the official-style
# continuous playout cursor, protected 10-frame observation tail, and local
# policy ACK gate.  The legacy gear_sonic_deploy copy is retained only for old
# standalone tooling and must not be used by the main BXI sim2sim path.
export PYTHONPATH="${REPO_ROOT}/src/bxi_example_py_elf3:${REPO_ROOT}:${PYTHONPATH:-}"

if [[ -f .venv_teleop/bin/activate ]]; then
  # Official PICO/XRoboToolkit dependencies live in the teleop venv.
  source .venv_teleop/bin/activate
fi

PICO_HOST="${PICO_HOST:-127.0.0.1}"
PICO_PORT="${PICO_PORT:-5556}"
SMPL_REF_ZMQ_HOST="${SMPL_REF_ZMQ_HOST:-127.0.0.1}"
SMPL_REF_ZMQ_PORT="${SMPL_REF_ZMQ_PORT:-5557}"
SMPL_REF_ZMQ_TOPIC="${SMPL_REF_ZMQ_TOPIC:-smpl_ref}"
SMPL_REF_CONTROL_HOST="${SMPL_REF_CONTROL_HOST:-127.0.0.1}"
SMPL_REF_CONTROL_PORT="${SMPL_REF_CONTROL_PORT:-5558}"
SMPL_REF_CONTROL_TOPIC="${SMPL_REF_CONTROL_TOPIC:-smpl_ref_control}"
WRIST_SOURCE="${WRIST_SOURCE:-pico_g1_legacy}"
BRIDGE_LOG_EVERY="${BRIDGE_LOG_EVERY:-1}"
PICO_RAW_RECORD_PATH="${PICO_RAW_RECORD_PATH:-}"
PICO_RAW_RECORD_MAX_FRAMES="${PICO_RAW_RECORD_MAX_FRAMES:-30000}"

if [[ -z "${PICO_MANAGER_ARGS:-}" ]]; then
  PICO_MANAGER_ARGS=(
    --manager
    --num_frames_to_send 10
    --target_fps 50
    --cuda
    --port "${PICO_PORT}"
  )
  if [[ "${PICO_ENABLE_VIS:-0}" == "1" ]]; then
    PICO_MANAGER_ARGS+=(--vis_vr3pt --vis_smpl)
  fi
else
  # shellcheck disable=SC2206
  PICO_MANAGER_ARGS=(${PICO_MANAGER_ARGS})
fi

if [[ -n "${PICO_RAW_RECORD_PATH}" ]]; then
  PICO_MANAGER_ARGS+=(
    --raw_record_path "${PICO_RAW_RECORD_PATH}"
    --raw_record_max_frames "${PICO_RAW_RECORD_MAX_FRAMES}"
  )
fi

pico_pid=""
bridge_pid=""

cleanup() {
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
  if [[ -n "${pico_pid}" ]] && kill -0 "${pico_pid}" 2>/dev/null; then
    kill "${pico_pid}" 2>/dev/null || true
    wait "${pico_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[elf3-pico-sources] starting official PICO manager"
if [[ -n "${PICO_RAW_RECORD_PATH}" ]]; then
  echo "[elf3-pico-sources] raw PICO SDK capture will be written on shutdown: ${PICO_RAW_RECORD_PATH}"
fi
python3 gear_sonic/scripts/pico_manager_thread_server.py "${PICO_MANAGER_ARGS[@]}" &
pico_pid=$!

sleep 1.0

echo "[elf3-pico-sources] starting ACK-gated official-style continuous-cursor PICO pose -> ELF3 smpl_ref bridge"
python3 -m bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge \
  --pico-host "${PICO_HOST}" \
  --pico-port "${PICO_PORT}" \
  --pico-topic pose \
  --out-host "${SMPL_REF_ZMQ_HOST}" \
  --out-port "${SMPL_REF_ZMQ_PORT}" \
  --out-topic "${SMPL_REF_ZMQ_TOPIC}" \
  --control-host "${SMPL_REF_CONTROL_HOST}" \
  --control-port "${SMPL_REF_CONTROL_PORT}" \
  --control-topic "${SMPL_REF_CONTROL_TOPIC}" \
  --wrist-source "${WRIST_SOURCE}" \
  --log-every "${BRIDGE_LOG_EVERY}" &
bridge_pid=$!

sleep 0.2
if ! kill -0 "${bridge_pid}" 2>/dev/null; then
  set +e
  wait "${bridge_pid}"
  bridge_exit=$?
  set -e
  echo "[elf3-pico-sources] ERROR: bridge exited during startup rc=${bridge_exit}" >&2
  exit "${bridge_exit}"
fi
if ! kill -0 "${pico_pid}" 2>/dev/null; then
  set +e
  wait "${pico_pid}"
  pico_exit=$?
  set -e
  echo "[elf3-pico-sources] ERROR: PICO manager exited during startup rc=${pico_exit}" >&2
  exit "${pico_exit}"
fi

echo "[elf3-pico-sources] ready"
echo "  1. stand in the L-shape calibration pose"
echo "  2. press A+B+X+Y for CALIB_FULL / PLANNER"
echo "  3. align your whole body with the suspended robot"
echo "  4. use Terminal 2 controller to enter pd_brake -> normal -> sonic_teleop"
echo "  5. press A+X to enter POSE"
echo "     deploy will auto-release after controller state=sonic_teleop and POSE policy is driving"
echo "  stop: switch Terminal 2 out of sonic_teleop before PICO OFF or Ctrl-C; ordinary pose loss holds the last reference window"

exited_pid=""
set +e
wait -n -p exited_pid "${pico_pid}" "${bridge_pid}"
child_exit=$?
set -e
echo "[elf3-pico-sources] child exited pid=${exited_pid:-unknown} rc=${child_exit}; stopping sources"
exit "${child_exit}"
