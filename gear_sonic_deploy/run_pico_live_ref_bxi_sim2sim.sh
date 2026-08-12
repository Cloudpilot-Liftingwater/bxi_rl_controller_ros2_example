#!/usr/bin/env bash
# Compatibility entry point. The canonical SONIC sim2sim launcher lives in script/.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[sonic-bxi-sim2sim] legacy entry point; forwarding to script/run_sonic_bxi_sim2sim.sh"
exec bash "${REPO_ROOT}/script/run_sonic_bxi_sim2sim.sh" "$@"
