"""Hardware workflow offline: real input mapper, original B states and manual suite.

Feedback follows targets in memory; this verifies routing and completion, not
robot dynamics, motor hardware or physical gamepad button presses.
"""
import importlib
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from bxi_example_py_elf3.framework.joints import JointTargetBuffer
from bxi_example_py_elf3.framework.mod_api import ModDefinition, MotorFrame, PoseState
from bxi_example_py_elf3.framework.runtime.mod_loader import _compose_config, _discover_mods
from bxi_example_py_elf3.framework.runtime.state_builder import build_robot_states
from bxi_example_py_elf3.framework.runtime.state_machine import RemoteEventAdapter, RobotStateMachine
from bxi_example_py_elf3.framework.runtime.transition import register_transition_plugin
from test_confirmed_gamepad import confirmed_messages
from test_dance_execution import Logger, context
from test_execution_suite import module
from test_model_test_manual import setup_suite

ROOT = Path(__file__).resolve().parents[1]
BASIC = "com.bxi.basic_actions/"
SUSPENDED = "com.bxi.suspended_tests/"
SUITE = "com.bxi.execution_suite/"


@pytest.fixture(autouse=True)
def local_launch_logs(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))


@pytest.mark.parametrize("topic,enabled,mode,allowed", [
    ("hardware/", "1", "manual", True),
    ("hardware/", "0", "manual", False),
    ("hardware/", "1", "auto", False),
    ("simulation/", "0", "manual", True),
    ("other/", "1", "manual", False),
])
def test_pd_to_idle_uses_same_button_only_for_explicit_hardware_suite(
        monkeypatch, topic, enabled, mode, allowed):
    monkeypatch.setenv("BXI_EXECUTION_SUITE", enabled)
    monkeypatch.setenv("BXI_EXECUTION_SUITE_MODE", mode)
    states = module("mods/com.bxi.suspended_tests/control_states.py", "unified_idle_gate")
    session = states.TestSession()
    idle = states.SuspendedIdleState(SUSPENDED + "idle", 1, session)
    ctx = NS(ros_node=NS(topic_prefix=topic, runtime=NS(current_state_name=BASIC + "pd_brake")))
    assert idle.is_available(ctx) is allowed
    session.faulted = True
    assert not idle.is_available(ctx)


def test_complete_hardware_button_workflow_keeps_original_300_second_vibration(
        monkeypatch, confirmed_messages):
    _, suite, model_states, ctx, _, _ = setup_suite(monkeypatch, "hardware/")
    package = ModuleType("_unified_hardware_suspended")
    package.__path__ = [str(ROOT / "mods/com.bxi.suspended_tests")]
    monkeypatch.setitem(sys.modules, package.__name__, package)
    suspended_plugin = importlib.import_module(package.__name__ + ".plugin")
    original = importlib.import_module(package.__name__ + ".control_states")
    monkeypatch.setattr(original, "get_package_share_path", lambda name: ROOT)
    suspended = suspended_plugin.create_mod(NS(mod_root=ROOT / "mods/com.bxi.suspended_tests"))

    clock = NS(value=100.0)
    for mod in (original, sys.modules["_elf3_model_test_thermal_policy"]):
        monkeypatch.setattr(mod, "time", NS(monotonic=lambda: clock.value))
    # Use the real model state factories, with small in-memory policies.
    suite = ModDefinition(
        state_factories={name: lambda build, name=name: model_states[name] for name in model_states},
        remote_event_filter=suite.remote_event_filter,
        state_observer=suite.state_observer,
        transition_plugins=suite.transition_plugins,
    )
    for transition in suite.transition_plugins.values():
        register_transition_plugin(transition)
    normal_target = JointTargetBuffer(ctx.robot_layout)
    normal_target.update(np.zeros(29), np.full(29, 10.0), np.ones(29))
    model_states["ground_prepare"].normal.get().default_target = normal_target.view

    class Stand(PoseState):
        def target_position(self, ctx): return np.zeros(29, np.float32)
        def gains(self, ctx): return np.full(29, 10., np.float32), np.ones(29, np.float32)

    def stand_factory(build):
        for name, value in build.params.items(): build.param(name, type(value), value)
        return Stand(build.name, build.state_id)

    mods = _discover_mods((ROOT / "mods",))
    definitions = {name: ModDefinition(state_factories={
        state: stand_factory for state in mod.manifest["states"]
    }) for name, mod in mods.items()}
    definitions.update({"com.bxi.suspended_tests": suspended, "com.bxi.execution_suite": suite})
    config, factories = _compose_config(
        yaml.safe_load((ROOT / "config/elf3_state_machine.yaml").read_text()),
        tuple(mods.values()), definitions,
    )
    states = build_robot_states(config, factories)
    for state in states.values(): state._bind_logger(Logger())
    ctx.last_motor_frame = MotorFrame.create(ctx.robot_layout, np.zeros(29), np.ones(29), np.ones(29))

    def resolve(frame, out):
        return out.update(frame.qpos, frame.kp, frame.kd, vel=frame.vel, torque=frame.torque)

    ctx.resolve_motor_frame = resolve
    ctx.set_motor_target = lambda frame: resolve(frame, ctx.last_motor_frame)
    ctx.preheat_model = lambda policy: (policy.reset(ctx.inference_frame), policy.step(ctx.inference_frame, .02, advance=False))
    sm = RobotStateMachine(ctx, config, states, logger=Logger())
    ctx.state_machine = sm
    ctx.request_state = lambda target, **kwargs: sm.request_transition(target, **kwargs)
    ctx.ros_node.runtime = NS(current_state_name=sm.current_state_name)
    adapter = RemoteEventAdapter(config["remote_events"])
    visited = []
    vibration_entered = None
    vibration_elapsed = None

    def tick(message=None):
        nonlocal vibration_entered, vibration_elapsed
        clock.value += .02
        ctx.actuator_temperatures.received_at = clock.value
        ctx.robot_joints.timestamp_ns += 1
        ctx.robot_joints.position[:] = ctx.last_motor_frame.qpos
        events = [] if message is None else adapter.extract_events(message)
        if message is not None:
            events = suspended.remote_event_filter(message, events, sm.current_state_name)
            events = suite.remote_event_filter(message, events, sm.current_state_name)
        transition = sm.update(.02, events)
        if not transition: sm.update_current_state(.02)
        ctx.ros_node.runtime.current_state_name = sm.current_state_name
        suspended.state_observer(sm.current_state_name)
        suite.state_observer(sm.current_state_name)
        if not visited or visited[-1] != sm.current_state_name:
            if visited and visited[-1] == SUSPENDED + "sequence_vibration":
                vibration_elapsed = clock.value - vibration_entered
            visited.append(sm.current_state_name)
            if sm.current_state_name == SUSPENDED + "sequence_vibration": vibration_entered = clock.value

    def press(name):
        for phase, message in confirmed_messages[name]:
            if phase == "repeat": break
            tick(message)

    def wait_for(condition, maximum=25000):
        for _ in range(maximum):
            if condition(): return
            tick()
        raise AssertionError((sm.current_state_name, visited))

    assert sm.current_state_name == BASIC + "pd_brake"
    press("idle_p0_r0")  # LB+B, without an intermediate zero-torque request.
    idle = states[SUSPENDED + "idle"]
    wait_for(lambda: idle.session.ready)
    assert any("READY for B sequence" in line for line in idle.logger.messages)
    press("sequence")  # B down then up, produced by the actual C++ mapper.
    running = states[SUSPENDED + "sequence_running"]
    wait_for(lambda: sm.current_state_name == running.name and running.phase == "playback")
    assert vibration_elapsed >= 300.0
    press("continue_p0_r0")  # RB+Y.
    wait_for(lambda: sm.current_state_name == BASIC + "normal" and not sm.in_transition)
    assert ctx.ros_node.topic_prefix == "hardware/"
    assert not hasattr(ctx.ros_node, "_simulation_release")  # Never wait for or request a virtual rope.

    for stage in ("dance", "sonic06", "sonic11"):
        # Temperatures in the model guard use the same virtual clock as the original B test.
        press(stage + "_p0_r0")
        wait_for(lambda: model_states[stage].result == "playback_complete"
                 and sm.current_state_name == BASIC + "normal" and not sm.in_transition)
        for _ in range(50): tick()
        assert sm.current_state_name == BASIC + "normal"
    session = model_states["ground_prepare"].session
    assert session.base_completed and session.completed == ["dance", "sonic06", "sonic11"]
    assert not session.active and not session.faulted and not idle.session.faulted
    assert visited == [BASIC + "pd_brake", SUSPENDED + "idle", SUSPENDED + "sequence_joint",
                       SUSPENDED + "sequence_vibration", SUSPENDED + "sequence_running",
                       SUITE + "ground_prepare", BASIC + "normal", SUITE + "dance", BASIC + "normal",
                       SUITE + "sonic06", BASIC + "normal", SUITE + "sonic11", BASIC + "normal"]


def launch_module(filename):
    return module("launch/" + filename, "unified_launch_" + filename.replace(".", "_"))


def test_hardware_launcher_reuses_original_controller_and_exact_sim_gamepad(monkeypatch, tmp_path):
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    launch = launch_module("example_launch_execution_suite_hw.launch.py")
    remote = ROOT.parent / "remote_controller"
    monkeypatch.setattr(launch, "get_package_share_path", lambda name: ROOT if name == "bxi_example_py_elf3" else remote)
    monkeypatch.setattr(launch, "Node", lambda **kwargs: NS(**kwargs))
    context = LaunchContext()
    sonic = tmp_path / "sonic"
    cxk = tmp_path / "cxk"
    for root, files in ((sonic, ("policy.py", "assets/sonic_action_token.onnx", "assets/stream_reference.npz")),
                        (cxk, ("assets/cxkv5_15k.onnx", "assets/cxkv5.npz"))):
        for filename in files:
            path = root / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    context.launch_configurations.update(sonic_root=str(sonic), cxk_root=str(cxk), start_remote_controller="true")
    original, gamepad = launch._suite_actions(context)
    assert isinstance(original, IncludeLaunchDescription)
    arguments = dict(original.launch_arguments)
    assert arguments["execution_suite"] == arguments["execution_eval"] == "1"
    assert "--disable-system-actions" in gamepad.arguments
    assert gamepad.arguments[-1] == str(remote / "config/execution_suite_gamepad.yaml")
    assert "simulation_release_on_normal" not in arguments


@pytest.mark.parametrize("sonic,cxk", [("", ""), ("/nonexistent-sonic", "/nonexistent-cxk")])
def test_missing_hardware_assets_fail_before_original_hardware_launch(monkeypatch, sonic, cxk):
    from launch import LaunchContext
    launch = launch_module("example_launch_execution_suite_hw.launch.py")
    monkeypatch.setattr(launch, "get_package_share_path", lambda name: ROOT)
    monkeypatch.setattr(launch, "IncludeLaunchDescription", lambda *a, **kw: pytest.fail("hardware included before asset validation"))
    context = LaunchContext()
    context.launch_configurations.update(sonic_root=sonic, cxk_root=cxk)
    with pytest.raises((ValueError, FileNotFoundError)):
        launch._suite_actions(context)


def test_original_hardware_launcher_explicitly_selects_manual_hardware_without_virtual_release(monkeypatch):
    from launch.actions import DeclareLaunchArgument
    launch = launch_module("example_demo_hw.launch.py")
    monkeypatch.setattr(launch, "_acquire_lock", lambda: None)
    monkeypatch.setattr(launch, "get_package_share_path", lambda name: ROOT)
    monkeypatch.setattr(launch, "Node", lambda **kwargs: NS(**kwargs))
    description = launch.generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, NS)]
    assert len(nodes) == 1 and nodes[0].executable == "bxi_example_py_elf3_demo"
    assert nodes[0].additional_env["BXI_EXECUTION_SUITE_MODE"] == "manual"
    assert {"BXI_EXECUTION_SUITE", "BXI_SONIC_ROOT", "BXI_CXK_ROOT", "ELF3_EVAL_ENABLE"} <= nodes[0].additional_env.keys()
    assert {"/topic_prefix": "hardware/"} in nodes[0].parameters
    assert all("/simulation_release_on_normal" not in parameter for parameter in nodes[0].parameters)
    assert any(isinstance(entity, DeclareLaunchArgument) and entity.name == "execution_suite"
               for entity in description.entities)
