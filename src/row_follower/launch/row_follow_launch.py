from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('row_follower')
    world = os.path.join(pkg, 'world', 'crops.world')

    percy_yaml = os.path.join(pkg, 'config', 'perception.yaml')
    ctrl_yaml  = os.path.join(pkg, 'config', 'controller.yaml')

    gz = ExecuteProcess(
        cmd=['gazebo', world, '--verbose'],
        output='screen'
    )

    percy = Node(
        package='row_follower',
        executable='perception',
        name='perception',
        output='screen',
        parameters=[percy_yaml] if os.path.isfile(percy_yaml) else []
    )

    ctrl = Node(
        package='row_follower',
        executable='controller',
        name='row_controller',
        output='screen',
        parameters=[ctrl_yaml] if os.path.isfile(ctrl_yaml) else []
    )

    return LaunchDescription([gz, percy, ctrl])
