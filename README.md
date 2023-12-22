# px-ros-pkg for ROS2

## Description
Original code from: <a>http://wiki.ros.org/px4flow_node </a><br><br>
A repository for PIXHAWK open source code running on ROS. There was no ROS2 implementation of the px4flow_node and the
official repository seems to be dead. Hence, I ported it to ROS 2.

## Supported Platforms
Tested on Ubuntu 22.04 with ROS Humble. Other ROS2 distributions and Ubuntu releases <i>might</i> work.

## Prerequisites:
- Boost library: ```sudo apt install libboost-dev```
- Eigen3: ```sudo apt install libeigen3-dev```
- ROS Humble (or others): <a>https://docs.ros.org/en/humble/Installation.html </a>

## Installation:
Put the whole folder into your ROS workspace and run<br>
```colcon build && . install/setup.bash```<br><br>
Preferably, add this repository as a submodule:<br>
```git submodule add -b main https://github.com/masskro0/px4flow_node_ros2 src/px4flow_node```

## Usage:
First, connect your px4flow sensor and then either launch the mavlink_serial_client:<br>
```ros2 launch mavlink_serial_client mavlink_launch.py```<br><br>
or the px4flow node:<br>
```ros2 launch px4flow px4flow_launch.py```

Both nodes use messages from the <b>px_comm</b> folder. Also, you can modify configuration parameters of both nodes
in <i>drivers/<pkg<pkg>>/config/default.yaml</i>