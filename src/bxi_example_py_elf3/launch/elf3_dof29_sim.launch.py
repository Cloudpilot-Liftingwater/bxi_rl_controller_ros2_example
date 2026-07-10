"""Official ELF3 29-DoF MuJoCo simulation-only launch.

FILE ROLE / SOURCE OF TRUTH:
This launch is part of the official BXI ELF3 sim2sim package. When sim2sim
model/process files conflict with GR00T-side standalone probes, prefer files
under:
  /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/

It starts the official 29-DoF torque-motor MJCF at
``data/mujoco_simulation/elf3.xml``. Keep this separate from GR00T training
assets.
"""

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import json

def generate_launch_description():

    xml_file_name = "data/mujoco_simulation/elf3.xml"
    xml_file = os.path.join(get_package_share_path("bxi_example_py_elf3"), xml_file_name)

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
        ]
    )
