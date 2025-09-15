from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('row_follower')
    world = os.path.join(pkg, 'world', 'crops.world')

    gz = ExecuteProcess(
        cmd=['gazebo', world, '--verbose'],
        output='screen'
    )

    perception = Node(
        package='row_follower',
        executable='perception',
        name='perception',
        output='screen'
    )

    controller = Node(
        package='row_follower',
        executable='controller',
        name='controller',
        output='screen'
    )

    return LaunchDescription([gz, perception, controller])

