# Installation

## ROS2

This repository is based on ROS2 Iron, which you can install [here](https://docs.ros.org/en/iron/Installation.html)

## Libraries

You will need to install the following libraries in order to launch the demos :

- [Stage Simulator and Stage ROS2](https://github.com/tuw-robotics/stage_ros2/blob/humble/res/install.md)
- [domain_bridge](https://github.com/ros2/domain_bridge?tab=readme-ov-file#installation) library
    ```bash
    sudo apt install ros-$ROS_DISTRO-domain-bridge
    ```
- [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?tab=readme-ov-file#linux-debian)
- [people_msgs](https://github.com/wg-perception/people/tree/ros2) :
    ```bash
    git clone https://github.com/wg-perception/people.git -b ros2
    colcon build --packages-select people_msgs
    ```
- [nav2_social_costmap_plugin](https://github.com/robotics-upo/nav2_social_costmap_plugin)
    ```bash
    git clone https://github.com/robotics-upo/nav2_social_costmap_plugin.git -b humble
    colcon build --packages-select nav2_social_costmap_plugin
    ```