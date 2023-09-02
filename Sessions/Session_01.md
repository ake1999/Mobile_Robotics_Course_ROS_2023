# Session 1

## Table of Contents

- [ROS 2 Distributions](#ros-2-distributions)
- [ROS 2 Foxy Installation](#ros-2-foxy-installation)
  - [Environment Setup](#environment-setup)
- [Creating a workspace](#creating-a-workspace)
- [ROS 2 CLI tools](#ros-2-cli-tools)

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
alias ctb='ssh ubuntu@192.168.2.74'   # Connect to Turtlebot3 (modify the IP address)

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
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash   # Source colcon_argcomplete for tab completion

export ROS_DOMAIN_ID=30           # Set ROS domain ID
```

You can safely change the domain ID for values between 0 to 101. For larger IDs, refer to the [ROS 2 Concepts: About Domain ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html) for more information.

If you want to use the local network only, you can set ROS_LOCALHOST_ONLY to 1. However, as we want to connect to Turtlebot later, you don't need to do that.

```bash
export ROS_LOCALHOST_ONLY=1
```

To know more about configuring the environment, you can refer to the [ROS 2 Beginner CLI tools: Configuring Environment](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

Also to know more about colcon and colcon_cd, please refer to the [ROS 2 Beginner Client libraries: Useing colcon to build packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

now Its time to create the ROS2 workspace

## Creating a Workspace

To create a workspace, follow these steps:

1. First, refer to the tutorial on building packages using colcon: [ROS 2 Beginner Client Libraries: Using colcon to Build Packages](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

2. After following the tutorial, check that three directories have been created in the workspace root:

   - `build`: This directory contains intermediate compilation files, tests, and temporary files.
   - `install`: This directory holds the compilation results along with all the necessary files for execution (specific configuration files, node startup scripts, maps, etc.). When building the workspace using the `--symlink-install` option, it creates symbolic links to their original locations (in `src` or `build`) instead of copying. This approach saves space and allows modifications to certain configuration files directly in `src`.
   - `log`: This directory contains a log of the compilation or testing process.

3. Going deeper into colcon:

   - `colcon` (collective construction) is a command-line tool used for building, testing, and using multiple software packages. It automates the process of building and sets up the environment to use the packages.

For more information, you can explore these resources:

- [ROS 2 Design: Build Tool](https://design.ros2.org/articles/build_tool.html)
- [colcon Documentation](https://colcon.readthedocs.io/)
- [Video: ROSCon 2019 - Using colcon in ROS 2](https://vimeopro.com/osrfoundation/roscon-2019/video/379127725)

## ROS 2 CLI Tools

In this session, we will dive into the world of ROS 2 Command Line Interface (CLI) tools. As you are already familiar with ROS 1, transitioning to ROS 2 CLI tools should be a smooth experience. We'll explore various aspects of working with ROS 2 CLI tools and get hands-on experience with the following materials:

1. **Using Turtlesim, ros2, and rqt:** We will start with the basics, including launching the Turtlesim simulator, using `ros2` command-line tools, and working with ROS 2's graphical tool, `rqt`.

2. **Understanding Nodes:** Nodes are fundamental units of computation in ROS 2. We'll learn how to create, manage, and interact with nodes.

3. **Understanding Topics:** Topics enable communication between nodes. We'll explore how to publish and subscribe to topics, allowing nodes to exchange data.

4. **Understanding Services:** Services provide a request-response mechanism between nodes. We'll learn how to define and use services for specific tasks.

5. **Understanding Parameters:** ROS 2 allows you to store and retrieve parameters, making it easy to configure and tune your robotic applications. We'll delve into the use of parameters.

6. **Understanding Actions:** Actions are a powerful way to perform long-running tasks in ROS 2. We'll understand how to define, create, and use actions.

7. **Using rqt_console to View Logs:** Effective debugging is crucial in robotics development. We'll explore how to use `rqt_console` to view logs and diagnose issues in your ROS 2 applications.

8. **Launching Nodes:** We'll learn how to launch nodes and manage multiple nodes using launch files, a convenient way to streamline your ROS 2 workflow.

9. **Recording and Playing Back Data:** Data recording and playback are essential for testing and analysis. We'll see how to record and replay data for evaluation and debugging.

To get started with ROS 2 CLI tools, refer to the [ROS 2 Beginner CLI Tools Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html) in the ROS 2 Foxy Documentation. This tutorial provides step-by-step instructions and hands-on exercises to help you become proficient with ROS 2 CLI tools.

As we progress through this session, you'll gain valuable skills for developing and debugging robotic applications using ROS 2. Let's explore these powerful tools together!
