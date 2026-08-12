#!/usr/bin/env bash
# Point the versioned SONIC runtime back to the previous verified release.
set -Eeuo pipefail

INSTALL_ROOT="${SONIC_INSTALL_ROOT:-${HOME}/sonic-elf3}"
COMPAT_PATH="${SONIC_COMPAT_PATH:-${HOME}/bxi_rl_controller_ros2_example-main}"

while (($#)); do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --compat-path)
      COMPAT_PATH="$2"
      shift 2
      ;;
    -h|--help)
      printf 'Usage: %s [--install-root DIR] [--compat-path PATH]\n' "$0"
      exit 0
      ;;
    *)
      printf '[rollback] unknown option: %s\n' "$1" >&2
      exit 2
      ;;
  esac
done

active_pattern='hardware_elf3|bxi_example_py_elf3_demo|run_sonic_pico_sources|pico_manager_thread_server|pico_pose_to_smpl_ref_bridge'
if pgrep -af "${active_pattern}" >/dev/null; then
  printf '[rollback] active robot/T3 processes detected; stop them first\n' >&2
  pgrep -af "${active_pattern}" >&2 || true
  exit 3
fi

[[ -L "${INSTALL_ROOT}/previous" ]] || {
  printf '[rollback] no previous release is recorded under %s\n' "${INSTALL_ROOT}" >&2
  exit 4
}
previous_target="$(readlink -f "${INSTALL_ROOT}/previous")"
[[ -d "${previous_target}" ]] || {
  printf '[rollback] previous release is missing: %s\n' "${previous_target}" >&2
  exit 4
}

current_target=""
if [[ -L "${INSTALL_ROOT}/current" ]]; then
  current_target="$(readlink -f "${INSTALL_ROOT}/current")"
fi

ln -s "${previous_target}" "${INSTALL_ROOT}/current.rollback"
mv -Tf "${INSTALL_ROOT}/current.rollback" "${INSTALL_ROOT}/current"
if [[ -n "${current_target}" ]]; then
  ln -sfn "${current_target}" "${INSTALL_ROOT}/previous"
fi
if [[ ! -e "${COMPAT_PATH}" ]]; then
  ln -s "${INSTALL_ROOT}/current" "${COMPAT_PATH}"
fi

printf '[rollback] current -> %s\n' "${previous_target}"
