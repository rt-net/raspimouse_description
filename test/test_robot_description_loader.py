# The MIT License (MIT)
#
# Copyright 2023 RT Corporation <support@rt-net.jp>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import math

from launch.launch_context import LaunchContext
import pytest

from raspimouse_description.robot_description_loader import \
    RobotDescriptionLoader


def exec_load(loader):
    # Command substitutionの実行方法はCommandのテストを参考にした
    # https://github.com/ros2/launch/blob/074cd2903ddccd61bce8f40a0f58da0b7c200481/launch/test/launch/substitutions/test_command.py#L47
    context = LaunchContext()
    return loader.load().perform(context)


def test_load_description():
    # xacroの読み込みが成功することを期待
    rdl = RobotDescriptionLoader()
    assert exec_load(rdl)


def test_change_description_path():
    # xacroのファイルパスを変更し、読み込みが失敗することを期待
    rdl = RobotDescriptionLoader()
    rdl.robot_description_path = 'hoge'
    with pytest.raises(Exception) as e:
        exec_load(rdl)
    assert e.value


def test_lidar_none():
    # lidarが変更されて、xacroに何もセットされないことを期待
    rdl = RobotDescriptionLoader()
    rdl.lidar = 'none'
    assert 'laser' not in exec_load(rdl)


def test_lidar_urg():
    # lidarが変更されて、xacroにURGがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.lidar = 'urg'
    assert 'urg_mount_link' in exec_load(rdl)


def test_lidar_lds():
    # lidarが変更されて、xacroにLDSがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.lidar = 'lds'
    assert 'lds_multi_mount_link' in exec_load(rdl)


def test_lidar_rplidar():
    # lidarが変更されて、xacroにRPLiDARがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.lidar = 'rplidar'
    assert 'rplidar_multi_mount_link' in exec_load(rdl)


def test_lidar_frame():
    # lidar_frameが変更されて、xacroにlaserがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.lidar = 'urg'
    rdl.lidar_frame = 'laser'
    assert 'laser' in exec_load(rdl)


def test_use_gazebo():
    # use_gazeboが変更され、xacroにgazebo_ros2_controlがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.gz_control_config_package = 'raspimouse_description'
    rdl.gz_control_config_file_path = 'test/dummy_controllers.yaml'
    assert 'gz_ros2_control/GazeboSimSystem' in exec_load(rdl)


def test_use_rgb_camera():
    # use_rgb_cameraが変更され、xacroにRGB Cameraがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_rgb_camera = 'true'
    rdl.gz_control_config_package = 'raspimouse_description'
    rdl.gz_control_config_file_path = 'test/dummy_controllers.yaml'
    assert 'realsense2_description/meshes/d435.dae' in exec_load(rdl)


def test_camera_link():
    # use_gazeboとuse_rgb_cameraが変更され、xacroにcamera linkがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.use_rgb_camera = 'true'
    rdl.gz_control_config_package = 'raspimouse_description'
    rdl.gz_control_config_file_path = 'test/dummy_controllers.yaml'
    assert 'camera_link' in exec_load(rdl)


def test_camera_downward():
    # camera_downwardが変更され、カメラが斜め30度下を向くことを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.use_rgb_camera = 'true'
    rdl.camera_downward = 'true'
    rdl.gz_control_config_package = 'raspimouse_description'
    rdl.gz_control_config_file_path = 'test/dummy_controllers.yaml'
    camera_angle = math.radians(30)
    assert '<pose>0 0 0 0 ' + str(camera_angle) + ' 0</pose>' in exec_load(rdl)
