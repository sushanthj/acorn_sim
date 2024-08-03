#!/bin/bash

#! NOTE: This script runs commands as sudo from root directory
# exit if any command below fails
set -e

# Check if ROS is sourced

if [ "$ROS_DISTRO" == "" ];
then
	echo "Installation cannot continue. No ROS sourced, please check if ROS is installed and sourced. Please try again after that!"
	exit 0
fi

sudo apt update

# Install build tool
sudo apt install python3-colcon-common-extensions

sudo apt-get install ros-humble-xacro #! change to ROS_DISTRO

# Install Gazebo packages

sudo apt-get install -y ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-gazebo-ros-pkgs

#Install xterm

sudo apt install -y xterm

#Install tmux
sudo apt install -y tmux

# Install navigation packages

#? Nav2
sudo apt install -y ros-$ROS_DISTRO-navigation2
sudo apt install -y ros-$ROS_DISTRO-nav2-*
sudo apt install -y ros-$ROS_DISTRO-turtlebot3-gazebo # for turtlebot3 simulation only
sudo apt install -y ros-$ROS_DISTRO-robot-localization
sudo apt install -y ros-$ROS_DISTRO-mapviz
sudo apt install -y ros-$ROS_DISTRO-mapviz-plugins
sudo apt install -y ros-$ROS_DISTRO-multires-image
sudo apt install -y ros-$ROS_DISTRO-tile-map
# sudo apt install -y ros-$ROS_DISTRO-slam-toolbox

#Teleop-joy
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-joy

#Teleop-key
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard

# rviz
sudo apt install -y ros-$ROS_DISTRO-rviz2

# clone git repos here...
# git clone --branch $ROS_DISTRO     https://github.com/username/package.git

# # build workspace
# cd ..
# colcon build --symlink-install

echo "export LC_NUMERIC="en_US.UTF-8" " >> ~/.bashrc

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "Setup Successful, Continue Installation after creating container!!!"

# echo "Do not forget to set the environment variables"

exit 0
