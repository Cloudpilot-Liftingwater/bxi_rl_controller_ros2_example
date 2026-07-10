"""Official ELF3 ROS2 sim2sim launch.

FILE ROLE / SOURCE OF TRUTH:
For ELF3 sim2sim, this BXI package is the authority when it conflicts with
GR00T-side standalone smoke/probe files:
  /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/

This launch starts the official MuJoCo simulation node with
``data/mujoco_simulation/elf3.xml`` and a controller node using the
``simulation/`` topic prefix. Do not replace GR00T training assets to run this
chain; keep sim2sim separate from training.
"""

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    xml_file_name = "data/mujoco_simulation/elf3.xml"
    xml_file = os.path.join(get_package_share_path("bxi_example_py_elf3"), xml_file_name)
    state_machine_config = os.path.join(
        get_package_share_path("bxi_example_py_elf3"),
        "config/elf3_state_machine.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),

            Node(
                package="bxi_example_py_elf3",
                executable="bxi_example_py_elf3_demo",
                name="bxi_example_py_elf3_demo",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/state_machine_config": state_machine_config},
                    {"/hot_reload": True},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
        ]
    )
