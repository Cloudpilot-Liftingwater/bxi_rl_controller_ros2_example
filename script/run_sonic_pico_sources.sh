#!/usr/bin/env bash
# T3: start PICO manager and bridge PICO pose stream to ELF3 smpl_ref stream.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BXI_RUNTIME_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${BXI_RUNTIME_ROOT}"
export BXI_RUNTIME_ROOT
export PYTHONPATH="${BXI_RUNTIME_ROOT}/src/bxi_example_py_elf3:${BXI_RUNTIME_ROOT}:${PYTHONPATH:-}"

if [[ -f .venv_teleop/bin/activate ]]; then
  source .venv_teleop/bin/activate
fi

PICO_HOST="${PICO_HOST:-127.0.0.1}"
PICO_PORT="${PICO_PORT:-5556}"
SMPL_REF_ZMQ_HOST="${SMPL_REF_ZMQ_HOST:-127.0.0.1}"
SMPL_REF_ZMQ_PORT="${SMPL_REF_ZMQ_PORT:-5557}"
SMPL_REF_ZMQ_TOPIC="${SMPL_REF_ZMQ_TOPIC:-smpl_ref}"
WRIST_SOURCE="${WRIST_SOURCE:-pico_g1_legacy}"
BRIDGE_LOG_EVERY="${BRIDGE_LOG_EVERY:-1}"

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

echo "[sonic-pico-sources] starting legacy PICO manager wrapper"
python3 -m bxi_example_py_elf3.sonic_pico.pico_manager_legacy "${PICO_MANAGER_ARGS[@]}" &
pico_pid=$!

sleep 1.0

echo "[sonic-pico-sources] starting PICO pose -> ELF3 smpl_ref bridge"
python3 -m bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge \
  --pico-host "${PICO_HOST}" \
  --pico-port "${PICO_PORT}" \
  --pico-topic pose \
  --out-host "${SMPL_REF_ZMQ_HOST}" \
  --out-port "${SMPL_REF_ZMQ_PORT}" \
  --out-topic "${SMPL_REF_ZMQ_TOPIC}" \
  --wrist-source "${WRIST_SOURCE}" \
  --log-every "${BRIDGE_LOG_EVERY}" &
bridge_pid=$!

echo "[sonic-pico-sources] ready"
echo "  controller flow: pd_brake -> normal -> sonic_teleop"
echo "  PICO mode: CALIB_FULL / PLANNER, then POSE"
echo "  wrist_source=${WRIST_SOURCE} (ELF3 FK path is prepared but not default yet)"

wait "${pico_pid}"
