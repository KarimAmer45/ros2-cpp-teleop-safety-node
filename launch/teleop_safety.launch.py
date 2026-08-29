# Copyright 2026 Karim Amer
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('ros2_cpp_teleop_safety_node')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=PathJoinSubstitution(
                    [package_share, 'config', 'teleop_safety.yaml']
                ),
            ),
            Node(
                package='ros2_cpp_teleop_safety_node',
                executable='teleop_safety_node',
                name='teleop_safety_node',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )
