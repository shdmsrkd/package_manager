#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('package_manager'),
        'config',
        'packages.yaml'
    )

    package_manager_node = Node(
        package='package_manager',
        executable='package_manager',
        name='package_manager_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_dir]
    )

    return LaunchDescription([package_manager_node])
