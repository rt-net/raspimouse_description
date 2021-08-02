# raspimouse_description

ROS package with URDF description macro for [Raspberry Pi Mouse](https://rt-net.jp/products/raspberrypimousev3/)

![display_launch](https://rt-net.github.io/images/raspberry-pi-mouse/display_launch.png)

This ROS package was separated from [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim).  
See details from [rt-net/raspimouse_sim#42](https://github.com/rt-net/raspimouse_sim/pull/42).

## Requirements

- ROS 2
  - [Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html)

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
