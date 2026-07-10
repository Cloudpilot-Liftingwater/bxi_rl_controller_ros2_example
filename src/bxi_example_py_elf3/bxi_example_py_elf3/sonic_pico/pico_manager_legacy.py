"""Temporary entry point for the legacy PICO manager during ELF3 FK migration.

The final runtime should move the PICO manager itself into this package and use
ELF3 FK calibration.  This wrapper keeps the verified PICO startup path working
while the new ELF3 FK path is A/B tested.
"""

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
        "Cannot locate legacy gear_sonic/scripts/pico_manager_thread_server.py. "
        "Set BXI_RUNTIME_ROOT to the BXI runtime root."
    )


def main() -> int:
    runtime_root = _resolve_runtime_root()
    legacy_script = runtime_root / "gear_sonic" / "scripts" / "pico_manager_thread_server.py"
    sys.path.insert(0, str(runtime_root))
    sys.argv[0] = str(legacy_script)
    runpy.run_path(str(legacy_script), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
