#!/bin/sh

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

export ROS_LOCALHOST_ONLY=1
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

echo " Installing turtlesim"
sudo apt install ros-humble-turtlesim

echo " Installing rqt "
sudo apt install ~nros-humble-rqt*

ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

ros2 node list

ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

ros2 node info /my_turtle

rqt_graph

ros2 topic echo /turtle1/cmd_vel

ros2 topic info /turtle1/cmd_vel

ros2 interface show geometry_msgs/msg/Twist

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic hz /turtle1/pose

ros2 service type /clear

ros2 service list -t

ros2 service find std_srvs/srv/Empty

ros2 interface show turtlesim/srv/Spawn

ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
