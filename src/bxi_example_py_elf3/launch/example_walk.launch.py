"""Official ELF3 ROS2 sim2sim walk launch.

FILE ROLE / SOURCE OF TRUTH:
This launch is part of the official BXI ELF3 sim2sim package. If sim2sim
model/process files conflict with GR00T-side standalone probes, prefer:
  /home/huangchenwei/ros2_ws/bxi_rl_controller_ros2_example-main/src/bxi_example_py_elf3/

It starts the official MuJoCo simulation model and MJLab/walk controller with
the ``simulation/`` topic prefix. Keep this separate from GR00T training
assets.
"""

import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    xml_file_name = "data/mujoco_simulation/elf3.xml"
    xml_file = os.path.join(get_package_share_path("bxi_example_py_elf3"), xml_file_name)
    
    onnx_file_name = "data/mjlab_model/model_normal.onnx"
    onnx_file = os.path.join(get_package_share_path("bxi_example_py_elf3"), onnx_file_name)

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
                executable="bxi_example_py_elf3_mjlab",
                name="bxi_example_py_elf3_mjlab",
                output="screen",
                parameters=[
                    {"/topic_prefix": "simulation/"},
                    {"/onnx_file": onnx_file},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
            ),
        ]
    )
