#!/usr/bin/env bash
# Build a self-contained, offline SONIC ELF3 deployment bundle for Ubuntu 22.04 x86_64.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

OUTPUT_DIR="${RUNTIME_ROOT}/dist"
WHEELHOUSE="${SONIC_WHEELHOUSE:-/data/tmp/pico_wheels}"
XRT_ROOT="${SONIC_XRT_ROOT:-/data/tmp/pico_extra}"
RELEASE_ID=""
SOURCE_REF="HEAD"
INCLUDE_OFFLINE_DEPS=1

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "" \
    "Options:" \
    "  --output-dir DIR       Output directory (default: ${OUTPUT_DIR})" \
    "  --release-id ID        Release identifier (default: timestamp + git SHA)" \
    "  --source-ref REF       Exact Git commit/ref to package (default: HEAD)" \
    "  --wheelhouse DIR       Offline Python wheels (default: ${WHEELHOUSE})" \
    "  --xrt-root DIR         XRT deb/SDK staging directory (default: ${XRT_ROOT})" \
    "  --no-offline-deps      Build source-only bundle" \
    "  -h, --help             Show this help"
}

while (($#)); do
  case "$1" in
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --release-id)
      RELEASE_ID="$2"
      shift 2
      ;;
    --source-ref)
      SOURCE_REF="$2"
      shift 2
      ;;
    --wheelhouse)
      WHEELHOUSE="$2"
      shift 2
      ;;
    --xrt-root)
      XRT_ROOT="$2"
      shift 2
      ;;
    --no-offline-deps)
      INCLUDE_OFFLINE_DEPS=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '[bundle] unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

for command_name in git python3 tar sha256sum patchelf; do
  command -v "${command_name}" >/dev/null || {
    printf '[bundle] missing required command: %s\n' "${command_name}" >&2
    exit 3
  }
done

git_commit="$(git -C "${RUNTIME_ROOT}" rev-parse --verify "${SOURCE_REF}^{commit}")"
git_sha="${git_commit:0:12}"
if [[ -z "${RELEASE_ID}" ]]; then
  RELEASE_ID="$(date +%Y%m%d_%H%M%S)-${git_sha}"
fi
if [[ ! "${RELEASE_ID}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  printf '[bundle] invalid release id: %s\n' "${RELEASE_ID}" >&2
  exit 4
fi

tracked_dirty=0
if [[ -n "$(git -C "${RUNTIME_ROOT}" status --porcelain --untracked-files=no)" ]]; then
  tracked_dirty=1
fi
worktree_dirty=0
if [[ -n "$(git -C "${RUNTIME_ROOT}" status --porcelain)" ]]; then
  worktree_dirty=1
fi

work_dir="$(mktemp -d /tmp/sonic-elf3-bundle.XXXXXX)"
cleanup() {
  rm -rf "${work_dir}"
}
trap cleanup EXIT

release_tree="${work_dir}/runtime"
bundle_root="${work_dir}/sonic-elf3-${RELEASE_ID}-ubuntu22-amd64"
mkdir -p "${bundle_root}/payload" "${bundle_root}/deploy" "${bundle_root}/python" \
  "${bundle_root}/vendor"

export_commit_file() {
  local relative_path="$1"
  local destination="$2"
  local mode="${3:-0644}"
  local object="${git_commit}:${relative_path}"

  if ! git -C "${RUNTIME_ROOT}" cat-file -e "${object}" 2>/dev/null; then
    printf '[bundle] required file is not tracked by source commit %s: %s\n' \
      "${git_commit}" "${relative_path}" >&2
    exit 5
  fi
  mkdir -p "$(dirname "${destination}")"
  git -C "${RUNTIME_ROOT}" cat-file blob "${object}" >"${destination}"
  chmod "${mode}" "${destination}"
}

printf '[bundle] generating safe runtime tree (protected advanced actions removed)\n'
commit_sanitizer="${work_dir}/sanitize_release.py"
export_commit_file "tools/sanitize_release.py" "${commit_sanitizer}" 0755
(
  cd "${RUNTIME_ROOT}"
  python3 "${commit_sanitizer}" \
    --source-ref "${git_commit}" \
    --out "${release_tree}" \
    --self-check
)

# The sanitizer must export the immutable commit, never the current worktree.
if [[ -d "${release_tree}/.git" ]]; then
  printf '[bundle] .git unexpectedly present in release tree\n' >&2
  exit 5
fi

# Transitional and generated material is not part of the production runtime.
rm -rf \
  "${release_tree}/.github" \
  "${release_tree}/gear_sonic_deploy" \
  "${release_tree}/history" \
  "${release_tree}/tools" \
  "${release_tree}/.venv_teleop"

robot_states="${release_tree}/src/bxi_example_py_elf3/bxi_example_py_elf3/robot_states.py"
if rg -q 'sonic teleop orientation unsafe' "${robot_states}"; then
  printf '[bundle] SONIC orientation gate is still present in release tree\n' >&2
  exit 5
fi
if ! rg -q 'check safe error, zero_torque' "${robot_states}"; then
  printf '[bundle] global orientation protection disappeared unexpectedly\n' >&2
  exit 5
fi

tar --sort=name --mtime='UTC 2020-01-01' --owner=0 --group=0 --numeric-owner \
  -C "${release_tree}" -czf "${bundle_root}/payload/runtime-src.tar.gz" .

cp "${release_tree}/script/install_sonic_deploy_bundle.sh" "${bundle_root}/deploy/install.sh"
cp "${release_tree}/script/verify_sonic_deploy.sh" "${bundle_root}/deploy/verify.sh"
cp "${release_tree}/script/rollback_sonic_deploy.sh" "${bundle_root}/deploy/rollback.sh"
cp "${release_tree}/script/sonic_pico_requirements.txt" "${bundle_root}/python/requirements.txt"
chmod +x "${bundle_root}/deploy/"*.sh

if ((INCLUDE_OFFLINE_DEPS)); then
  [[ -d "${WHEELHOUSE}" ]] || {
    printf '[bundle] wheelhouse not found: %s\n' "${WHEELHOUSE}" >&2
    exit 6
  }
  for wheel_pattern in \
    'numpy-1.26.4-*.whl' \
    'scipy-1.15.3-*.whl' \
    'pyzmq-27.1.0-*.whl' \
    'msgpack-1.1.2-*.whl' \
    'torch-2.6.0+cpu-*.whl' \
    'pin-2.7.0-*.whl' \
    'onnx-1.22.0-*.whl' \
    'onnxruntime-1.23.2-*.whl'; do
    compgen -G "${WHEELHOUSE}/${wheel_pattern}" >/dev/null || {
      printf '[bundle] required wheel missing: %s/%s\n' "${WHEELHOUSE}" "${wheel_pattern}" >&2
      exit 6
    }
  done
  mkdir -p "${bundle_root}/python/wheels"
  cp "${WHEELHOUSE}/"*.whl "${bundle_root}/python/wheels/"
  # Keep the robot-validated versions pinned in requirements.txt. These newer
  # resolver downloads are intentionally excluded from the offline index.
  rm -f \
    "${bundle_root}/python/wheels/numpy-2.2.6-"*.whl \
    "${bundle_root}/python/wheels/sympy-1.14.0-"*.whl \
    "${bundle_root}/python/wheels/typing_extensions-4.16.0-"*.whl

  xrt_deb="${XRT_ROOT}/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb"
  xrt_sdk="${XRT_ROOT}/xrobotoolkit_pybind/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
  xrt_lib="${XRT_ROOT}/xrobotoolkit_pybind/lib/libPXREARobotSDK.so"
  for required_path in "${xrt_deb}" "${xrt_sdk}" "${xrt_lib}"; do
    [[ -f "${required_path}" ]] || {
      printf '[bundle] XRT component missing: %s\n' "${required_path}" >&2
      exit 7
    }
  done
  mkdir -p "${bundle_root}/vendor/xrt"
  cp "${xrt_deb}" "${bundle_root}/vendor/xrt/roboticsservice_1.0.0.0_amd64.deb"
  cp "${xrt_sdk}" "${bundle_root}/vendor/xrt/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
  cp "${xrt_lib}" "${bundle_root}/vendor/xrt/libPXREARobotSDK.so"
  patchelf --set-rpath '$ORIGIN' \
    "${bundle_root}/vendor/xrt/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so"
fi

model_rel="src/bxi_example_py_elf3/data/sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx"
reference_rel="src/bxi_example_py_elf3/data/sonic_reference/elf3_pico_stand_clean_001/stream_reference.npz"
stand_reference_rel="src/bxi_example_py_elf3/data/sonic_reference/elf3_pico_stand_clean_001/stream_reference.npz"
model_sha="$(sha256sum "${release_tree}/${model_rel}" | awk '{print $1}')"
reference_sha="$(sha256sum "${release_tree}/${reference_rel}" | awk '{print $1}')"
stand_reference_sha="$(sha256sum "${release_tree}/${stand_reference_rel}" | awk '{print $1}')"

{
  printf 'RELEASE_ID=%q\n' "${RELEASE_ID}"
  printf 'SOURCE_COMMIT=%q\n' "${git_commit}"
  printf 'SOURCE_EXPORT_MODE=%q\n' 'git-commit'
  printf 'SOURCE_TRACKED_DIRTY=%q\n' "${tracked_dirty}"
  printf 'SOURCE_WORKTREE_DIRTY=%q\n' "${worktree_dirty}"
  printf 'TARGET_OS=%q\n' 'ubuntu22.04'
  printf 'TARGET_ARCH=%q\n' 'x86_64'
  printf 'TARGET_PYTHON=%q\n' '3.10'
  printf 'ROS_DOMAIN_ID=%q\n' '22'
  printf 'ADVANCED_ACTIONS_INCLUDED=%q\n' '0'
  printf 'OFFLINE_DEPS_INCLUDED=%q\n' "${INCLUDE_OFFLINE_DEPS}"
  printf 'SONIC_MODEL_SHA256=%q\n' "${model_sha}"
  printf 'SONIC_REFERENCE_SHA256=%q\n' "${reference_sha}"
  printf 'SONIC_STAND_REFERENCE_SHA256=%q\n' "${stand_reference_sha}"
} >"${bundle_root}/manifest.env"

(
  cd "${bundle_root}"
  find . -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum >SHA256SUMS
)

mkdir -p "${OUTPUT_DIR}"
archive="${OUTPUT_DIR}/$(basename "${bundle_root}").tar.gz"
tar --sort=name --mtime='UTC 2020-01-01' --owner=0 --group=0 --numeric-owner \
  -C "${work_dir}" -czf "${archive}" "$(basename "${bundle_root}")"
sha256sum "${archive}" >"${archive}.sha256"

printf '[bundle] created: %s\n' "${archive}"
printf '[bundle] checksum: %s\n' "${archive}.sha256"
printf '[bundle] release: %s commit=%s tracked_dirty=%s\n' \
  "${RELEASE_ID}" "${git_commit}" "${tracked_dirty}"
