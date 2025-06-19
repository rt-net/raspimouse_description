# raspimouse_description

[![industrial_ci](https://github.com/rt-net/raspimouse_description/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/raspimouse_description/actions/workflows/industrial_ci.yml)

ROS package with URDF description macro for [Raspberry Pi Mouse](https://rt-net.jp/products/raspberrypimousev3/)

![display_launch](https://rt-net.github.io/images/raspberry-pi-mouse/display_launch.png)

This ROS package was separated from [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim).
See details from [rt-net/raspimouse_sim#42](https://github.com/rt-net/raspimouse_sim/pull/42).

## Table of Contents

- [raspimouse\_description](#raspimouse_description)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
    - [ROS 2](#ros-2)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Binary Installation](#binary-installation)
    - [Source Build](#source-build)
  - [QuickStart](#quickstart)
  - [License](#license)
  - [Contributing](#contributing)

## Supported ROS distributions

### ROS 2

- Humble ([`humble`](https://github.com/rt-net/raspimouse_description/tree/humble))
- Jazzy ([`jazzy`](https://github.com/rt-net/raspimouse_description/tree/jazzy))

## Requirements

- Raspberry Pi Mouse
  - [Summary](https://rt-net.jp/products/raspberrypimousev3/)
  - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=4141)
- Linux OS
  - Ubuntu Desktop 24.04
- ROS 2
  - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Installation

### Binary Installation

```bash
sudo apt install ros-$ROS_DISTRO-raspimouse-description
```

### Source Build

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone package
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse_description

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## QuickStart

Display a Raspberry Pi Mouse robot model on RViz2 with the following command:

```sh
ros2 launch raspimouse_description display.launch.py
```

You can also display a LiDAR mounted robot model with the following command:

```sh
ros2 launch raspimouse_description display.launch.py lidar:=rplidar
```

The `lidar` option supports `urg`, `lds`, and `rplidar`.

Similarly, display an RGB Camera mounted robot model with the following command:

```sh
ros2 launch raspimouse_description display.launch.py use_rgb_camera:=true
```

![mouse_with_rgb_camera.png](https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rgb_camera.png)

The RGB Camera can be pointed down with the following command:

```sh
ros2 launch raspimouse_description display.launch.py use_rgb_camera:=true camera_downward:=true
```

![mouse_with_rgb_camera_downward.png](https://rt-net.github.io/images/raspberry-pi-mouse/mouse_with_rgb_camera_downward.png)

## License

(C) 2016-2022 RT Corporation \<support@rt-net.jp\>

Each file is licensed as stated in their headers.  
If no license is specified, the file is licensed under the Apache License, Version 2.0.  
The full license text is available in the [LICENSE](./LICENSE) file or at [https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0).

### Acknowledgements

Special thanks to https://gbiggs.github.io/rosjp_urdf_tutorial_text/index.html

The file [robotis_lds01.stl](./meshes/stl/robotis_lds01.stl) is released from ROBOTIS and licensed under the [Apache License 2.0](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/LICENSE).
The original file is released in [turtlebot3_description package](https://github.com/ROBOTIS-GIT/turtlebot3/blob/a3c515b350a752b93ed8de4a009442e80e9d787d/turtlebot3_description/meshes/sensors/lds.stl).

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md).