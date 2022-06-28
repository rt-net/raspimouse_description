# Copyright 2020 RT Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", "lds", or "rplidar".')
    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Set lidar link name.')
    declare_arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Set namespace for tf tree.')
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')

    xacro_file = os.path.join(
        get_package_share_directory('raspimouse_description'),
        'urdf',
        'raspimouse.urdf.xacro')
    params = {'robot_description': Command(['xacro ', xacro_file,
                                            ' lidar:=', LaunchConfiguration('lidar'),
                                            ' lidar_frame:=', LaunchConfiguration('lidar_frame'),
                                            ]),
              'frame_prefix': [LaunchConfiguration('namespace'), '/']}

    push_ns = PushRosNamespace([LaunchConfiguration('namespace')])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[params])

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    rviz_config_file = get_package_share_directory(
        'raspimouse_description') + '/launch/config/urdf.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        declare_arg_lidar,
        declare_arg_lidar_frame,
        declare_arg_namespace,
        declare_arg_use_rviz,
        push_ns,
        rsp,
        jsp,
        rviz_node,
        ])