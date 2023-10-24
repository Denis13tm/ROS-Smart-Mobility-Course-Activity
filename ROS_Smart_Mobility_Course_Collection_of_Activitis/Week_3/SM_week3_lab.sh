#!/bin/sh

#----------- Setting locale-------------------

echo  " ### Setting locale ### "

locale  # check for UTF-8

sudo apt update && sudo apt install locales (Commented to prevent actual changes)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Setting up sources
echo " ### Adding ROS2 apt repository to your system ### "

sudo apt install software-properties-common
sudo add-apt-repository universe

echo " ### Adding ROS2 GPG key with apt ### "

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo " ### Installing ROS2 packages ### "

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

echo " ### Trying some examples ### "
echo " ### Talker-listener ### "

#--------------- C++ talker---------------
source /opt/ros/humble/setup.bash 
ros2 run demo_nodes_cpp talker

# ----------------Python listener--------------
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener

echo " ### The end ### "
