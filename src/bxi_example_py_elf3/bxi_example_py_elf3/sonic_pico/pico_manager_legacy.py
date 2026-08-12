"""Entry point for the ELF3-native PICO manager shipped with this runtime."""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


def _resolve_runtime_root() -> Path:
    env_root = os.environ.get("BXI_RUNTIME_ROOT")
    if env_root:
        return Path(env_root).resolve()

    cwd = Path.cwd().resolve()
    if (cwd / "gear_sonic" / "scripts" / "pico_manager_thread_server.py").exists():
        return cwd

    for parent in Path(__file__).resolve().parents:
        if (parent / "gear_sonic" / "scripts" / "pico_manager_thread_server.py").exists():
            return parent

    raise FileNotFoundError(
        "Cannot locate gear_sonic/scripts/pico_manager_thread_server.py. "
        "Set BXI_RUNTIME_ROOT to the BXI runtime root."
    )


def main() -> int:
    runtime_root = _resolve_runtime_root()
    manager_script = runtime_root / "gear_sonic" / "scripts" / "pico_manager_thread_server.py"
    sys.path.insert(0, str(runtime_root))
    sys.argv[0] = str(manager_script)
    runpy.run_path(str(manager_script), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
