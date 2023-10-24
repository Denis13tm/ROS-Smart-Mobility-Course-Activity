#!/bin/bash

# Navigate to the cpp_pubsub package directory
cd ~/ros2_ws/src/cpp_pubsub

# Download the example talker code with topic statistics
wget -O src/member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp

# Modify CMakeLists.txt to include the new executable
cat <<EOL >> CMakeLists.txt

# Add the listener_with_topic_statistics executable
add_executable(listener_with_topic_statistics src/member_function_with_topic_statistics.cpp)
ament_target_dependencies(listener_with_topic_statistics rclcpp std_msgs)

install(TARGETS
  talker
  listener
  listener_with_topic_statistics
  DESTINATION lib/\${PROJECT_NAME})
EOL

# Modify package.xml to add dependencies
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:build_depend" -t elem -v "rclcpp" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:build_depend" -t elem -v "std_msgs" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:exec_depend" -t elem -v "rclcpp" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:exec_depend" -t elem -v "std_msgs" package.xml

# Build the ROS 2 workspace
cd ~/ros2_ws
colcon build --symlink-install

# Run the subscriber with topic statistics enabled
ros2 run cpp_pubsub listener_with_topic_statistics

# Open a new terminal and run the talker node
# Optionally, you can run this in the background
ros2 run cpp_pubsub talker

# View the available topics
ros2 topic list

# Observe the published statistics data
ros2 topic echo /statistics


