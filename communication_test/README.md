# Implementation demo of multi robot communication with ROS2

This package is a ROS2 package that implements the different solutions explained in the repository's root README.

## 0. Dependencies

- [ROS2 Iron](https://docs.ros.org/en/iron/Installation.html)
- [Stage Simulator and Stage ROS2](https://github.com/tuw-robotics/stage_ros2/blob/humble/res/install.md)
- [domain_bridge]https://github.com/ros2/domain_bridge?tab=readme-ov-file#installation library
- [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?tab=readme-ov-file#linux-debian)
- [people_msgs](https://github.com/wg-perception/people/tree/ros2) :
```bash
git clone https://github.com/wg-perception/people.git -b src/people
colcon build --packages-select people_msgs
```
- [nav2_social_costmap_plugin](https://github.com/robotics-upo/nav2_social_costmap_plugin)
```bash
git clone https://github.com/robotics-upo/nav2_social_costmap_plugin.git -b src/nav2_social_costmap_plugin
colcon build --packages-select nav2_social_costmap_plugin
```


## 1. Demos

In the following READMEs you'll find all the commands to run the demos :

- [Simulation demos](docs/simulation.md)
- [Real robots demos](docs/real.md)
