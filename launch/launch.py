# Copyright 2025 WheelHub Intelligent
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='')
    reset_arg = DeclareLaunchArgument('reset', default_value='false')

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_imu'),
        'config',
        'imu_hardware_jy61p.yaml'
    ])

    # Node definition
    start_whi_imu_node = Node(
        package='whi_imu',
        executable='whi_imu_node',
        name='whi_imu',
        parameters=[
            config_file,
            {'reset_z': LaunchConfiguration('reset')}, # always available
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_name_arg,
        reset_arg,
        start_whi_imu_node
    ])
