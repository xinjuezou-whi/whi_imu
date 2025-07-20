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
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='')
    reset_arg = DeclareLaunchArgument('reset', default_value='false')

    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('whi_imu'),
        'config',
        'imu_hardware_jy61p.yaml'
    )

    # Node definition
    whi_imu_node = Node(
        package='whi_imu',
        executable='whi_imu_node',
        name='whi_imu',
        output='screen',
        parameters=[
            config_file,
            {'reset_z': LaunchConfiguration('reset')}, # always available
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        reset_arg,
        whi_imu_node
    ])
