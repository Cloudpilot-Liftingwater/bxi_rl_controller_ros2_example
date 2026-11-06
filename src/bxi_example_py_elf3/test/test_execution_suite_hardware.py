"""Standalone entry uses ordinary config and original hardware launcher."""
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from test_execution_suite import module
ROOT=Path(__file__).resolve().parents[1]


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
    assert "--config" in gamepad.arguments
    assert gamepad.arguments[-1] == str(remote / "config/xbox_default.yaml")
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


@pytest.mark.parametrize('enabled', ['0', '1'])
def test_ordinary_hardware_start_owns_automatic_passive_recorder(monkeypatch, tmp_path, enabled):
    from launch import LaunchContext
    launch = launch_module('example_demo_hw.launch.py')
    monkeypatch.setattr(launch, 'ExecuteProcess', lambda **kw: NS(**kw))
    config = tmp_path/'robot.yaml'
    config.write_text("robot_id: '58'\n")
    context = LaunchContext()
    context.launch_configurations.update(execution_eval_auto_record=enabled, execution_eval='1',
        robot_config_file=str(config), execution_eval_output='/tmp/eval-runs', execution_eval_socket='test_socket')
    processes = launch._automatic_recording(context)
    assert len(processes) == int(enabled)
    if processes:
        process = processes[0]
        assert process.cmd[1:5] == ['-m', 'elf3_eval.cli', 'auto-record', '--output']
        assert 'ELF3-58' in process.cmd and 'test_socket' in process.cmd
        assert process.output == 'screen'  # existing *_elf.log redirection
        assert 'OPENBLAS_NUM_THREADS' in process.additional_env
