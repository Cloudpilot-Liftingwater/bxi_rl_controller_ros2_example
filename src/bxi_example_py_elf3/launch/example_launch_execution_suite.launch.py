"""Full robot_test simulation: official BXI simulator and one BXI controller."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,EmitEvent,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration,EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_path('bxi_example_py_elf3')
    simulator=Node(package='mujoco',executable='simulation',name='simulation_mujoco',
        output='screen',emulate_tty=True,arguments=[str(share/'data/elf3.xml')])
    controller=Node(package='bxi_example_py_elf3',executable='bxi_example_py_elf3_demo',
        name='bxi_example_py_elf3_demo',output='screen',emulate_tty=True,
        parameters=[{'/topic_prefix':'simulation/','/release_suspension':False,
                     '/simulation_release_on_normal':True}],
        additional_env={'BXI_EXECUTION_SUITE':'1','ELF3_EVAL_ENABLE':'1',
                        'BXI_EXECUTION_SUITE_MODE':LaunchConfiguration('execution_mode'),
                        'ELF3_EVAL_SOCKET':LaunchConfiguration('execution_eval_socket'),
                        'BXI_SONIC_ROOT':LaunchConfiguration('sonic_root'),
                        'BXI_CXK_ROOT':LaunchConfiguration('cxk_root')})
    return LaunchDescription([
        DeclareLaunchArgument('execution_mode',default_value=EnvironmentVariable('BXI_EXECUTION_SUITE_MODE',default_value='manual'),choices=['manual','auto']),
        DeclareLaunchArgument('execution_eval_socket',default_value='elf3_eval_mujoco_suite_v1'),
        DeclareLaunchArgument('sonic_root',default_value=EnvironmentVariable('BXI_SONIC_ROOT',default_value='/data/ros2_ws/com.bxi.sonic-main')),
        DeclareLaunchArgument('cxk_root',default_value=EnvironmentVariable('BXI_CXK_ROOT',default_value='/data/ros2_ws/备份/com.bxi.cxk')),
        RegisterEventHandler(OnProcessExit(target_action=simulator,on_exit=[EmitEvent(event=Shutdown(reason='simulation exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=[EmitEvent(event=Shutdown(reason='controller exited'))])),
        simulator,controller])
