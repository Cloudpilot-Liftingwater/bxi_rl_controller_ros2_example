"""Standalone model consistency test wrapper, reusing the original hardware launcher.

The gamepad profile is shared byte for byte with sim2sim. Hardware reset,
IMU selection and process exclusion remain owned by example_demo_hw.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from bxi_example_py_elf3.framework.mod_api.hardware_launch import declare_hardware_launch_arguments


def _suite_actions(context):
    share = get_package_share_path("bxi_example_py_elf3")
    roots = {}
    for argument in ("sonic_root", "cxk_root"):
        value = LaunchConfiguration(argument).perform(context).strip()
        if not value:
            raise ValueError(f"{argument} is required for the hardware execution suite")
        roots[argument] = Path(value).expanduser().resolve()

    # Resolve assets before including the launch that starts hardware. No
    # inference, reset, publisher or motor access occurs in this preparation.
    required = [
        roots["sonic_root"] / "policy.py",
        roots["sonic_root"] / "assets/sonic_action_token.onnx",
        roots["sonic_root"] / "assets/stream_reference.npz",
        roots["cxk_root"] / "assets/cxkv5_15k.onnx",
        roots["cxk_root"] / "assets/cxkv5.npz",
        share / "mods/com.bxi.execution_suite/assets/sonic06.npz",
        share / "mods/com.bxi.execution_suite/assets/sonic11.npz",
        share / "data/data.txt",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("Execution suite assets missing: " + ", ".join(missing))
    from elf3_eval import host  # Check the passive observer is importable before hardware starts.
    del host

    arguments = {name: LaunchConfiguration(name) for name in (
        "robot_config_file", "enable_head", "enable_imu", "imu_runtime_timeout_sec",
        "execution_eval_socket",
    )}
    arguments.update(
        execution_suite="1", execution_eval="1",
        **{name: str(path) for name, path in roots.items()},
    )
    original = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/example_demo_hw.launch.py")),
        launch_arguments=arguments.items(),
    )
    gamepad = Node(
        package="remote_controller", executable="remote_controller",
        name="remote_controller", output="screen", emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_remote_controller")),
        arguments=["--driver", "joystick", "--config",
                   str(get_package_share_path("remote_controller") / "config/xbox_default.yaml")],
    )
    return [original, gamepad]


def generate_launch_description():
    return LaunchDescription(declare_hardware_launch_arguments() + [
        DeclareLaunchArgument("sonic_root", default_value=EnvironmentVariable("BXI_SONIC_ROOT", default_value="")),
        DeclareLaunchArgument("cxk_root", default_value=EnvironmentVariable("BXI_CXK_ROOT", default_value="")),
        DeclareLaunchArgument("execution_eval_socket", default_value=EnvironmentVariable(
            "ELF3_EVAL_SOCKET", default_value="elf3_model_eval_v1")),
        DeclareLaunchArgument("imu_runtime_timeout_sec", default_value="0.1"),
        DeclareLaunchArgument("start_remote_controller", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_suite_actions),
    ])
