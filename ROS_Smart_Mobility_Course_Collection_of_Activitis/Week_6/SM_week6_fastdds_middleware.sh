#!/bin/bash

# Create a new ROS 2 workspace if it doesn't exist
if [ ! -d ~/ros2_ws ]; then
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp
fi

# Create the XML profile file
echo '<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- default publisher profile -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </publisher>

    <!-- default subscriber profile -->
    <subscriber profile_name="default_subscriber" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </subscriber>

    <!-- publisher profile for topic sync_topic -->
    <publisher profile_name="/sync_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>SYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

    <!-- publisher profile for topic async_topic -->
    <publisher profile_name="/async_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

</profiles>' > ~/ros2_ws/src/sync_async_node_example_cpp/SyncAsync.xml

# Build the ROS 2 package
cd ~/ros2_ws
colcon build

# Set up ROS 2 environment
source install/setup.bash

# Export environment variables for XML configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/sync_async_node_example_cpp/SyncAsync.xml

# Run the publisher node
ros2 run sync_async_node_example_cpp SyncAsyncWriter &
PUBLISHER_PID=$!

# Run the subscriber node
ros2 run sync_async_node_example_cpp SyncAsyncReader &
SUBSCRIBER_PID=$!

# Wait for the nodes to finish
wait $PUBLISHER_PID
wait $SUBSCRIBER_PID

