# raspimouse_description

[![industrial_ci](https://github.com/rt-net/raspimouse_description/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/raspimouse_description/actions/workflows/industrial_ci.yml)

ROS package with URDF description macro for [Raspberry Pi Mouse](https://rt-net.jp/products/raspberrypimousev3/)

![display_launch](https://rt-net.github.io/images/raspberry-pi-mouse/display_launch.png)

This ROS package was separated from [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim).
See details from [rt-net/raspimouse_sim#42](https://github.com/rt-net/raspimouse_sim/pull/42).

The main development branch for ROS 1 is [`master`](https://github.com/rt-net/raspimouse_description/tree/master).
The main development branch for ROS 2 is [`ros2`](https://github.com/rt-net/raspimouse_description/tree/ros2).

## Supported ROS distributions

- Humble ([`humble`](https://github.com/rt-net/raspimouse_description/tree/humble))
- Jazzy ([`jazzy`](https://github.com/rt-net/raspimouse_description/tree/jazzy))

## Installation

```sh
# Clone raspimouse_description and install dependencies
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_description
rosdep install -r -y -i --from-paths .

# Build the package
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## How to Use

Display a Raspberry Pi Mouse robot model on RViz2 with the following command:

```sh
ros2 launch raspimouse_description display.launch.py
```

You can also display a LiDAR mounted robot model with the following command:

```sh
ros2 launch raspimouse_description display.launch.py lidar:=rplidar
```

The `lidar` option supports `urg`, `lds`, and `rplidar`.

Similarly, display a RGB Camera mounted robot model with the following command:

```sh
ros2 launch raspimouse_description display.launch.py use_rgb_camera:=true
```
![](https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rgb_camera.png)

RGB Camera can be pointed down with the following command:

```sh
ros2 launch raspimouse_description display.launch.py use_rgb_camera:=true camera_downward:=true
```

![](https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rgb_camera_downward.png)


## LICENSE

(C) 2016-2022 RT Corporation

This repository is licensed under the MIT license, see [LICENSE](./LICENSE).
Unless attributed otherwise, everything in this repository is under the MIT license.

### Acknowledgements

Special thanks to https://gbiggs.github.io/rosjp_urdf_tutorial_text/index.html

The file [robotis_lds01.stl](./meshes/stl/robotis_lds01.stl) is released from ROBOTIS and licensed under the [Apache License 2.0](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/LICENSE).
The original file is released in [turtlebot3_description package](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/turtlebot3_description/meshes/sensors/lds.stl).
