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
cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspimouse_description
rosdep install -r -y -i --from-paths .

# Build the package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## How to Use

Display a Raspberry Pi Mouse robot model on RViz with the following comand:

```sh
roslaunch raspimouse_description display_xacro.launch 
```

## LICENSE

(C) 2016-2020 RT Corporation \<shop@rt-net.jp\>

This repository is licensed under the MIT license, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the MIT license.

### Acknowledgements

Special thanks to https://gbiggs.github.io/rosjp_urdf_tutorial_text/index.html
