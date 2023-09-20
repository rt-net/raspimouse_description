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
from raspimouse_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_rgb_camera = DeclareLaunchArgument(
        'use_rgb_camera',
        default_value='false',
        description='Set true to attach the rgb camera model.'
    )
    declare_use_line_sensor = DeclareLaunchArgument(
        'use_line_sensor',
        default_value='false',
        description='Set true to attach the line sensor model.'
    )
    declare_use_lidar_urg = DeclareLaunchArgument(
        'use_lidar_urg',
        default_value='false',
        description='Set true to attach the URG-04LX-UG01 model.'
    )
    declare_use_lidar_lds = DeclareLaunchArgument(
        'use_lidar_lds',
        default_value='false',
        description='Set true to attach the LDS-01 model.'
    )
    declare_use_lidar_rplidar = DeclareLaunchArgument(
        'use_lidar_rplidar',
        default_value='false',
        description='Set true to attach the RPLiDAR A1 model.'
    )
    declare_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Set true to attach the IMU model.'
    )
    declare_arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Set namespace for tf tree.'
    )
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_rgb_camera = LaunchConfiguration('use_rgb_camera')
    description_loader.use_line_sensor = LaunchConfiguration('use_line_sensor')
    description_loader.use_lidar_urg = LaunchConfiguration('use_lidar_urg')
    description_loader.use_lidar_lds = LaunchConfiguration('use_lidar_lds')
    description_loader.use_lidar_rplidar = LaunchConfiguration('use_lidar_rplidar')
    description_loader.use_imu = LaunchConfiguration('use_imu')

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
        declare_use_rgb_camera,
        declare_use_line_sensor,
        declare_use_lidar_urg,
        declare_use_lidar_lds,
        declare_use_lidar_rplidar,
        declare_use_imu,
        declare_arg_namespace,
        declare_arg_use_rviz,
        push_ns,
        rsp,
        jsp,
        rviz_node,
        ])
