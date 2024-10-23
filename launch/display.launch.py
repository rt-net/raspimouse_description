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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from raspimouse_description.robot_description_loader import RobotDescriptionLoader


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
    declare_arg_use_rgb_camera = DeclareLaunchArgument(
        'use_rgb_camera',
        default_value='false',
        description='Set "true" to mount rgb camera.')
    declare_arg_camera_downward = DeclareLaunchArgument(
        'camera_downward',
        default_value='false',
        description='Set "true" to point the camera downwards.')

    description_loader = RobotDescriptionLoader()
    description_loader.lidar = LaunchConfiguration('lidar')
    description_loader.lidar_frame = LaunchConfiguration('lidar_frame')
    description_loader.use_rgb_camera = LaunchConfiguration('use_rgb_camera')
    description_loader.camera_downward = LaunchConfiguration('camera_downward')

    push_ns = PushRosNamespace([LaunchConfiguration('namespace')])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': description_loader.load()}])

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
        declare_arg_use_rgb_camera,
        declare_arg_camera_downward,
        push_ns,
        rsp,
        jsp,
        rviz_node,
        ])
