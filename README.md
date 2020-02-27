# raspimouse_description

ROS package with URDF description macro for [Raspberry Pi Mouse](https://www.rt-net.jp/products/raspimouse2?lang=en)

![display_launch](https://github.com/rt-net/raspimouse_description/blob/image/display_launch.png)

## Requirements

- ROS
  - [Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - [Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)

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

This repository is licensed under the MIT license, see [LICENSE](./LICENSE).

Unless attributed otherwise, everything in this repository is under the MIT license.
