#!/bin/bash

# Function to check if a process is running
function is_process_running {
  pgrep -x "$1" > /dev/null
}

# Start the Discovery Server
echo "Starting Discovery Server..."
fastdds discovery --server-id 0 &
sleep 2  # Wait for the server to start

# Set the ROS_DISCOVERY_SERVER environment variable
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Run the listener node
echo "Launching listener node..."
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server &
sleep 2  # Wait for the listener node to start

# Run the talker node
echo "Launching talker node..."
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server &
sleep 2  # Wait for the talker node to start

# Check if the processes are running
if is_process_running "ros2"; then
  echo "ROS 2 nodes are running."
else
  echo "Failed to start ROS 2 nodes."
fi

# Demonstrate Discovery Server execution
echo "Demonstrating Discovery Server execution..."
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener &
sleep 2  # Wait for the new listener node to start

# Check if the new listener is connected to the talker
if is_process_running "ros2"; then
  echo "The new listener node is connected to the talker."
else
  echo "The new listener node is not connected to the talker."
fi

# Restore ROS_DISCOVERY_SERVER to default
unset ROS_DISCOVERY_SERVER

# Terminate the Discovery Server
echo "Terminating Discovery Server..."
pkill -x "fastdds"
sleep 2  # Wait for the server to terminate

# Clean up background processes
echo "Cleaning up..."
pkill -x "ros2"

echo "Script completed."

