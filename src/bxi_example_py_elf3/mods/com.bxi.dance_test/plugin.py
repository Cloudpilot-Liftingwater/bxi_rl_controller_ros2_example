"""On-demand dance resources; no second ROS actuator publisher."""
import os

from bxi_example_py_elf3.framework.mod_api import ModDefinition, ResourceKey
from .policy import load_dance_policy
from .state import DanceTestState
from bxi_example_py_elf3.model_test_safety import ModelTemperatureGuard

DANCE_POLICY = ResourceKey("com.bxi.dance_test/policy")


def _load_policy(context):
    # Both source and installed Mods have the same ../../data layout.
    data = context.mod_root.parent.parent / "data"
    fixed_pos = os.environ.get("BXI_DANCE_FIXED_POS", "false").lower()
    if fixed_pos not in ("true", "false"):
        raise ValueError("BXI_DANCE_FIXED_POS must be true or false")
    return load_dance_policy(
        os.environ.get("BXI_DANCE_MOTION", str(data / "dance.npz")),
        os.environ.get("BXI_DANCE_MODEL", str(data / "dance.onnx")),
        variant=os.environ.get("BXI_DANCE_VARIANT", "auto"),
        start_frame=int(os.environ.get("BXI_DANCE_START_FRAME", "0")),
        fixed_pos=fixed_pos == "true",
    )


def create_mod(context):
    context.register_resource(DANCE_POLICY, _load_policy, policy="on_demand")
    handle = context.resource(DANCE_POLICY)
    thermal = ModelTemperatureGuard(context.mod_root.parent/'com.bxi.suspended_tests/config')
    return ModDefinition(state_factories={
        "dance": lambda state: DanceTestState(state.name, state.state_id, handle, temperature_guard=thermal),
    })
