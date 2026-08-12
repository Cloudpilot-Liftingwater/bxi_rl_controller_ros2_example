#!/usr/bin/env bash
# Install a SONIC ELF3 bundle into a versioned directory and optionally activate it.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

INSTALL_ROOT="${SONIC_INSTALL_ROOT:-${HOME}/sonic-elf3}"
COMPAT_PATH="${SONIC_COMPAT_PATH:-${HOME}/bxi_rl_controller_ros2_example-main}"
ACTIVATE=0
INSTALL_XRT=0
DRY_RUN=0

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Options:" \
    "  --install-root DIR     Versioned install root (default: ${INSTALL_ROOT})" \
    "  --compat-path PATH     Existing runtime compatibility path (default: ${COMPAT_PATH})" \
    "  --activate             Activate after build/smoke; existing directory is backed up" \
    "  --install-xrt          Install bundled XRT PC service deb with sudo when missing" \
    "  --dry-run              Verify bundle and preflight only" \
    "  -h, --help             Show this help"
}

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
    --activate)
      ACTIVATE=1
      shift
      ;;
    --install-xrt)
      INSTALL_XRT=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[install] unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -f "${BUNDLE_ROOT}/manifest.env" ]] || {
  printf '[install] manifest.env is missing from bundle\n' >&2
  exit 3
}
[[ -f "${BUNDLE_ROOT}/SHA256SUMS" ]] || {
  printf '[install] SHA256SUMS is missing from bundle\n' >&2
  exit 3
}

printf '[install] verifying bundle checksums\n'
(cd "${BUNDLE_ROOT}" && sha256sum -c SHA256SUMS)

# shellcheck disable=SC1091
source "${BUNDLE_ROOT}/manifest.env"

[[ "$(uname -m)" == "${TARGET_ARCH}" ]] || {
  printf '[install] unsupported architecture: %s (expected %s)\n' \
    "$(uname -m)" "${TARGET_ARCH}" >&2
  exit 4
}
python_version="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
[[ "${python_version}" == "${TARGET_PYTHON}" ]] || {
  printf '[install] unsupported Python: %s (expected %s)\n' \
    "${python_version}" "${TARGET_PYTHON}" >&2
  exit 4
}
[[ -f /opt/ros/humble/setup.bash ]] || {
  printf '[install] ROS Humble is missing at /opt/ros/humble\n' >&2
  exit 4
}

bxi_prefix=""
for candidate in /opt/bxi/bxi_ros2_pkg /opt/bxi/bxi_ros2_pkg-main; do
  if [[ -f "${candidate}/local_setup.bash" || -f "${candidate}/setup.bash" ]]; then
    bxi_prefix="${candidate}"
    break
  fi
done
[[ -n "${bxi_prefix}" ]] || {
  printf '[install] BXI ROS package prefix was not found under /opt/bxi\n' >&2
  exit 4
}

if ((ACTIVATE)); then
  active_pattern='hardware_elf3|bxi_example_py_elf3_demo|run_sonic_pico_sources|pico_manager_thread_server|pico_pose_to_smpl_ref_bridge'
  if pgrep -af "${active_pattern}" >/dev/null; then
    printf '[install] active robot/T3 processes detected; stop them before --activate\n' >&2
    pgrep -af "${active_pattern}" >&2 || true
    exit 5
  fi
fi

required_kb=2097152
available_kb="$(df -Pk "$(dirname "${INSTALL_ROOT}")" | awk 'NR==2 {print $4}')"
if ((available_kb < required_kb)); then
  printf '[install] at least 2 GiB free space is required; available=%s KiB\n' \
    "${available_kb}" >&2
  exit 6
fi

printf '[install] release=%s commit=%s tracked_dirty=%s\n' \
  "${RELEASE_ID}" "${SOURCE_COMMIT}" "${SOURCE_TRACKED_DIRTY}"
printf '[install] install_root=%s compat_path=%s bxi_prefix=%s\n' \
  "${INSTALL_ROOT}" "${COMPAT_PATH}" "${bxi_prefix}"
if ((DRY_RUN)); then
  printf '[install] dry-run preflight passed\n'
  exit 0
fi

release_dir="${INSTALL_ROOT}/releases/${RELEASE_ID}"
[[ ! -e "${release_dir}" ]] || {
  printf '[install] release already exists: %s\n' "${release_dir}" >&2
  exit 7
}
mkdir -p "${INSTALL_ROOT}/releases" "${INSTALL_ROOT}/backups"
mkdir -p "${release_dir}"
tar -xzf "${BUNDLE_ROOT}/payload/runtime-src.tar.gz" -C "${release_dir}"

if [[ ! -x /opt/apps/roboticsservice/RoboticsServiceProcess ]]; then
  if ((! INSTALL_XRT)); then
    printf '[install] XRT PC service is missing; rerun with --install-xrt\n' >&2
    exit 8
  fi
  xrt_deb="${BUNDLE_ROOT}/vendor/xrt/roboticsservice_1.0.0.0_amd64.deb"
  [[ -f "${xrt_deb}" ]] || {
    printf '[install] bundled XRT deb is missing\n' >&2
    exit 8
  }
  sudo dpkg -i "${xrt_deb}"
fi

wheel_dir="${BUNDLE_ROOT}/python/wheels"
requirements="${BUNDLE_ROOT}/python/requirements.txt"
[[ -d "${wheel_dir}" && -f "${requirements}" ]] || {
  printf '[install] offline Python dependencies are not included in this bundle\n' >&2
  exit 9
}

printf '[install] creating release-local Python environment\n'
if ! /usr/bin/python3 -m venv --system-site-packages "${release_dir}/.venv_teleop"; then
  printf '[install] ensurepip unavailable; using system pip fallback\n'
  rm -rf "${release_dir}/.venv_teleop"
  /usr/bin/python3 -m venv --without-pip --system-site-packages \
    "${release_dir}/.venv_teleop"
fi
"${release_dir}/.venv_teleop/bin/python" -s -m pip install \
  --no-index --find-links "${wheel_dir}" -r "${requirements}"

site_dir="$("${release_dir}/.venv_teleop/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
install -m 0755 \
  "${BUNDLE_ROOT}/vendor/xrt/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" \
  "${site_dir}/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
install -m 0755 "${BUNDLE_ROOT}/vendor/xrt/libPXREARobotSDK.so" \
  "${site_dir}/libPXREARobotSDK.so"

printf '[install] building ROS packages with system Python\n'
(
  unset VIRTUAL_ENV PYTHONHOME
  export PATH='/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
  export PYTHONNOUSERSITE=1
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "${bxi_prefix}/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${bxi_prefix}/local_setup.bash"
  else
    # shellcheck disable=SC1090
    source "${bxi_prefix}/setup.bash"
  fi
  set -u
  cd "${release_dir}"
  colcon build --merge-install --packages-select bxi_example_py_elf3 remote_controller
)

bash "${BUNDLE_ROOT}/deploy/verify.sh" \
  --runtime "${release_dir}" \
  --bxi-prefix "${bxi_prefix}"

if ((ACTIVATE)); then
  timestamp="$(date +%Y%m%d_%H%M%S)"
  previous_target=""
  if [[ -L "${INSTALL_ROOT}/current" ]]; then
    previous_target="$(readlink -f "${INSTALL_ROOT}/current")"
  fi
  if [[ -e "${COMPAT_PATH}" && ! -L "${COMPAT_PATH}" ]]; then
    previous_target="${INSTALL_ROOT}/backups/pre-${RELEASE_ID}-${timestamp}"
    mv "${COMPAT_PATH}" "${previous_target}"
  fi
  if [[ -n "${previous_target}" ]]; then
    ln -sfn "${previous_target}" "${INSTALL_ROOT}/previous"
  fi
  ln -s "${release_dir}" "${INSTALL_ROOT}/current.new"
  mv -Tf "${INSTALL_ROOT}/current.new" "${INSTALL_ROOT}/current"
  if [[ -L "${COMPAT_PATH}" ]]; then
    ln -s "${INSTALL_ROOT}/current" "${COMPAT_PATH}.new"
    mv -Tf "${COMPAT_PATH}.new" "${COMPAT_PATH}"
  elif [[ ! -e "${COMPAT_PATH}" ]]; then
    ln -s "${INSTALL_ROOT}/current" "${COMPAT_PATH}"
  fi
  printf '[install] activated %s\n' "${release_dir}"
  printf '[install] rollback: bash %s/deploy/rollback.sh --install-root %s\n' \
    "${BUNDLE_ROOT}" "${INSTALL_ROOT}"
else
  printf '[install] release installed and verified but not activated\n'
  printf '[install] rerun with --activate after stopping robot/T3 processes\n'
fi
