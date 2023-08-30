#Session 1
## Table of Contents
- [ROS 2 Distributions](#ros-2-distributions)
- [ROS 2 Foxy Installation](#ros-2-foxy-installation)
  - [Environment Setup](#environment-setup)

## ROS 2 Distributions
As we aim to have both ROS 1 and ROS 2 coexisting on the same operating system, it's recommended to use Ubuntu 20.04 LTS. This version supports ROS 1 Noetic and ROS 2 Foxy, offering flexibility in using both versions side by side. Our initial focus will be on ROS 2 installation to get started with building our robotic applications.

Learn more about [ROS 2 Foxy and other distributions](https://docs.ros.org/en/foxy/Releases.html).

## ROS 2 Foxy Installation

To install ROS 2 Foxy, you can follow the instructions provided in the official ROS 2 documentation. Please visit the following link to start the installation process:

[ROS 2 Foxy Installation Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

However, we are going to do a slightly different **Environment Setup** during the installation. Please pause before proceeding with the installation on the official page and return here to set it up together.

## Environment Setup

To simplify your ROS 2 workflow, you can add the following lines to your `~/.bashrc` file:

```bash
alias ctb='ssh ubuntu@192.168.2.74'   # Connect to Turtlebot3

alias eb='nano ~/.bashrc'             # Edit .bashrc
alias vb='vim ~/.bashrc'              # Edit .bashrc with Vim
alias sb='source ~/.bashrc'           # Source .bashrc

alias cw='cd ~/catkin_ws'             # Change directory to catkin workspace
alias cs='cd ~/catkin_ws/src'         # Change directory to catkin source folder
alias cm='noetic && cd ~/catkin_ws && catkin_make'  # Source Noetic and build catkin workspace

alias fw='cd ~/ros2_ws'               # Change directory to ROS 2 workspace
alias fs='cd ~/ros2_ws/src'           # Change directory to ROS 2 source folder
alias cb='foxy && cd ~/ros2_ws && colcon build --symlink-install'     # Source Foxy and build ROS 2 workspace

alias noetic='source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash'  # Source Noetic environment
alias foxy='source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash'    # Source Foxy environment

export ROS_MASTER_URI=http://192.168.2.91:11311      # Set ROS master URI
export ROS_HOSTNAME=192.168.2.91                     # Set ROS hostname

export LDS_MODEL=LDS-02                             # Set LDS model
export TURTLEBOT3_MODEL=waffle_pi                    # Set Turtlebot3 model

source /usr/share/colcon_cd/function/colcon_cd.sh    # Source colcon_cd
export _colcon_cd_root=/opt/ros/foxy/                 # Set colcon_cd root
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash   # Source colcon_argcomplete

export ROS_DOMAIN_ID=30           # Set ROS domain ID