import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('aquaguard_hw'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(package='aquaguard_hw', executable='m3508_speed_node', name='m3508_speed_node',
             output='screen', emulate_tty=True, parameters=[params]),
    ])
