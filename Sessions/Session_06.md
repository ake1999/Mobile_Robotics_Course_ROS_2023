# Session 6: Exploring Nav 2 and Using the ROS 1 Bridge

## Table of Contents

- [Nav 2](#nav-2)
- [ROS 1 Bridge](#ros-1-bridge)
  - [Installing ROS 1 Bridge](#installing-ros-1-bridge)
  - [Running the ROS 1 Bridge](#running-the-ros-1-bridge)
  - [ROS 1 bridge example (Controlling a Turtlebot in Gazebo)](#ros-1-bridge-example-controlling-a-turtlebot-in-gazebo)

## Nav 2

To dive into the concepts and workings of Nav 2 (Navigation 2), the best resource is the official [Nav 2 Documentation](https://navigation.ros.org/index.html#). Make sure to explore this documentation to gain a deeper understanding of Nav 2 and its capabilities.

## ROS 1 Bridge

In this section, we'll introduce the ROS 1 Bridge, a crucial tool that enables seamless communication between ROS 1 and ROS 2 environments, making the migration process smoother and more accessible.

### Installing ROS 1 Bridge

Start by installing the ROS 1 Bridge package:

```bash
$ sudo apt install ros-foxy-ros1-bridge
```

### Running the ROS 1 Bridge

To start the ROS 1 Bridge, execute the following three commands:

```bash
$ noetic # or source /opt/ros/noetic/setup.bash
$ foxy   # or source /opt/ros/foxy/setup.bash
$ ros2 run ros1_bridge dynamic_bridge
```

### ROS 1 bridge example (Controlling a Turtlebot in Gazebo)

Now, let's control a Turtlebot in Gazebo using the ROS 1 Bridge. Follow these steps:

1. In Terminal 1, start ROS 1 (Noetic) and launch a Turtlebot3 simulation:

```bash
$ noetic
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

2. In Terminal 2, start ROS 2 (Foxy) and launch the teleoperation keyboard for Turtlebot3:

```bash
$ foxy
$ ros2 run turtlebot3_teleop teleop_keyboard
```

3. In Terminal 3, start both ROS 1 and ROS 2 and run the ROS 1 Bridge:

```bash
$ noetic && foxy
$ ros2 run ros1_bridge dynamic_bridge
```

Now, you can move the Turtlebot3 in Gazebo using the teleop_keyboard node in ROS 2 while controlling the simulation environment in ROS 1 Gazebo.

For more detailed information and usage instructions, refer to the [ROS 1 Bridge GitHub Repository](https://github.com/ros2/ros1_bridge/blob/master/README.md).

Explore the capabilities of the ROS 1 Bridge to bridge the gap between ROS 1 and ROS 2, enabling a smooth transition between the two ecosystems.
