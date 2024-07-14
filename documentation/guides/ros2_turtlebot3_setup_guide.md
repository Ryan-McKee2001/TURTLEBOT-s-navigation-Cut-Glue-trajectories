# ROS2 Installation and Setup with Turtlebot3 Packages

This guide provides a step-by-step process for setting up a ROS2 environment suitable for utilizing ROS2 packages developed within the scope of this project. While drawing inspiration from the Turtlebot3 quick start guide ([link](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)), it has been tailored to meet the specific requirements of this project, omitting any elements causing compatibility issues with the setup. For additional context, please refer to the quick start guide.

## Setting Up Ubuntu 22.04

Begin by setting up Ubuntu 22.04 either on VirtualBox or through a dual-boot configuration if your PC component drivers support Linux or WSL. Follow this [tutorial](https://medium.com/@maheshdeshmukh22/how-to-install-ubuntu-22-04-lts-on-virtualbox-in-windows-11-6c259ce8ef60).

Ensure that your virtual machine is configured with a bridged connection to enable LAN communication between your Turtlebot3 ROS2 and the remote server running Ubuntu 22.04.

![Bridged Connection Configuration](insert_image_link_here)

## Installing ROS2 and Setting Up ROS2 Humble Desktop

Open a terminal and execute the following commands:

```bash
locale  # Check for UTF-8 support

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # Verify settings


```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```bash
sudo apt install software-properties-common && sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```bash
sudo apt update && sudo apt install ros-dev-tools
```

```bash
sudo apt update && sudo apt upgrade
```

```bash
sudo apt install ros-humble-desktop
```

```bash
echo "source opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

First install vscode, atom text editor and terminator terminal these will aid in later ros2 development

Setup the navigation2 packages https://navigation.ros.org/getting_started/index.html#getting-started

Navigation2 contains the navigation2 stack packages for global and local planners as well as recovery behaviours and behavious tree interpreters which will make everything work together. (Link to explanation of nav2)
Nav2 bringup is a set of launch files, params.xmls which define behaviour trees for our navigation stack, world and robot models including turtlebot3 models for the robot and worlds we need this package will make implmeneting our own slam alogirhtms possible our planner packages work on top of this.

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

Setting up simulation and nav2_bringup configurations
This set of commands will set defaults for models in our gazebo simulation including using turtlebot3 burger as the default and using the turtlebot3_gazebo/models as the default package directory for models we can add to gazebo simulation
```bash
echo -e "\nexport TURTLEBOT3_MODEL=burger\nexport GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
Source .bashrc
```

Install cartographer SLAM packages

```bash
Sudo apt install ros-humble-cartographer
Sudo apt install ros-humble-cartographer-ros
```

You should now be able to ros2 launch the launch one of the several launch files for the navigation stack in nav2_bringup like so:

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

Building a ROS2 colcon workspace

Make the repo
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

Source and install and dependencies required for packages within the repo when packages are added 
```bash
cd ~/ros2_ws/ && 
rosdep install --from-paths src --ignore-src -r -y
```

Build the colcon environment

```bash
colcon build --symlink-install
```

then source the setup.bash for this repo so packages are recognized by ros2

```bash
echo -e "\n# Source ros2_ws\nsource ~/ros2_ws/install/setup.bash"
source "~/.bashrc"
```

You can now add or create any ros2 packages you want in the ~ros2_ws/src directory make sure to colcon build when you create new directory's sometimes if you make significant changes you will need to remove the rm -r -f install log directory in ~/ros2_ws and 

rebuild using colcon build --symlink-install 

You should now be able to work with ros2 humble and build packages specific for this project.





