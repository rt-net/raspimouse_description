# raspimouse_description

ROS package with URDF description macro for [Raspberry Pi Mouse](https://rt-net.jp/products/raspberrypimousev3/)

![display_launch](https://rt-net.github.io/images/raspberry-pi-mouse/display_launch.png)

This ROS package was separated from [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim).  
See details from [rt-net/raspimouse_sim#42](https://github.com/rt-net/raspimouse_sim/pull/42).

The main development branch for ROS 1 is [`master`](https://github.com/rt-net/raspimouse_description/tree/master).  
The main development branch for ROS 2 is [`ros2`](https://github.com/rt-net/raspimouse_description/tree/ros2).

## Supported ROS distributions

- ~~Kinetic ([`kinetic-devel`](https://github.com/rt-net/raspimouse_description/tree/kinetic-devel))~~ deprecated
- Melodic ([`melodic-devel`](https://github.com/rt-net/raspimouse_description/tree/melodic-devel))
- Foxy ([`foxy-devel`](https://github.com/rt-net/raspimouse_description/tree/foxy-devel))

## Installation

```sh
# Clone raspimouse_description and install dependencies
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse_description
rosdep install -r -y -i --from-paths .

# Build the package
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## How to Use

Display a Raspberry Pi Mouse robot model on RViz2 with the following comand:

```sh
ros2 launch raspimouse_description display.launch.py
```

## LICENSE

(C) 2016-2021 RT Corporation

This repository is licensed under the MIT license, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the MIT license.

### Acknowledgements

Special thanks to https://gbiggs.github.io/rosjp_urdf_tutorial_text/index.html

The file [robotis_lds01.stl](./meshes/stl/robotis_lds01.stl) is released from ROBOTIS and licensed under the [Apache License 2.0](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/LICENSE).
The original file is released in [turtlebot3_description package](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/turtlebot3_description/meshes/sensors/lds.stl).