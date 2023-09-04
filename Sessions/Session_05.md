# Session 5: Setting Up PC, Raspberry Pi, and Turtlebot3 with OpenManipulator

## Table of Contents

- [PC Setup](#pc-setup)
- [Raspberry Pi Setup](#raspberry-pi-setup)
- [Turtlebot3 with OpenManipulator Setup](#turtlebot3-with-openmanipulator-setup)

## PC Setup

Before we can start working with Turtlebot3 and OpenManipulator, we need to set up our PC. If you haven't already, install the required packages on your computer. However, keep in mind that we previously installed ROS 2 with a slightly different setup. You may need to adjust some commands to work properly with your ROS 2 installation.

Follow the instructions provided here for PC setup: [Turtlebot3 PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

## Raspberry Pi Setup

In this section, we'll cover setting up the Raspberry Pi for Turtlebot3. You have two options:

### Option 1: Fresh ROS Foxy Installation

1. If you've previously installed Ubuntu on your Raspberry Pi and want to start fresh, you can remove the existing Ubuntu installation.
2. After removal, you can burn a new image with ROS Foxy already installed by following the instructions provided here: [Turtlebot3 SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/).

### Option 2: ROS 2 Foxy Alongside ROS Noetic

1. If you'd like to keep your existing setup (e.g., Ubuntu with ROS Noetic) on your Raspberry Pi, you can install ROS 2 Foxy alongside it.
2. Begin from step 13 in the "Manual SBC Setup Instructions" provided here: [Turtlebot3 SBC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/).
3. Ensure you make similar changes as we did when installing ROS Foxy on your computer, and add them to the `.bashrc` file on your Raspberry Pi.

## Turtlebot3 with OpenManipulator Setup

Now that we've prepared both your PC and Raspberry Pi, it's time to set up Turtlebot3 with OpenManipulator. Follow the instructions in the link below to configure the robot and perform SLAM and navigation with it:

[Turtlebot3 with OpenManipulator Setup Instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation).

With these steps, you'll have your Turtlebot3 with OpenManipulator up and running, ready for further exploration and development.