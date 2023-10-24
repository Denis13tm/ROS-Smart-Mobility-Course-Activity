#!/bin/bash

# Step 1: Create a ROS 2 package
echo "Step 1: Creating a ROS 2 package..."
source /opt/ros/$ROS_DISTRO/setup.bash # Replace $ROS_DISTRO with your ROS distribution
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs
echo "ROS 2 package 'bag_recorder_nodes' created successfully."

# Step 1.1: Update package.xml
echo "Step 1.1: Updating package.xml..."
PACKAGE_XML="~/ros2_ws/src/bag_recorder_nodes/package.xml"
cat <<EOL > $PACKAGE_XML
<?xml version="1.0"?>
<package format="3">
  <name>bag_recorder_nodes</name>
  <version>0.0.0</version>
  <description>C++ bag writing tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>example_interfaces</build_depend>
  <build_depend>rclcpp</build_depend>
  <build_depend>rosbag2_cpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>launch_testing</test_depend>
  <export>
    <example_interfaces_msgs></example_interfaces_msgs>
  </export>
</package>
EOL
echo "package.xml updated successfully."

# Step 2: Write the C++ node
echo "Step 2: Writing the C++ node..."
mkdir -p ~/ros2_ws/src/bag_recorder_nodes/src
CPP_NODE="~/ros2_ws/src/bag_recorder_nodes/src/simple_bag_recorder.cpp"
cat <<EOL > $CPP_NODE
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>
using std::placeholders::_1;
class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");
    subscription_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
EOL
echo "C++ node 'simple_bag_recorder.cpp' created successfully."

# Step 2.2: Add executable to CMakeLists.txt
echo "Step 2.2: Updating CMakeLists.txt..."
CMAKELISTS="~/ros2_ws/src/bag_recorder_nodes/CMakeLists.txt"
cat <<EOL >> $CMAKELISTS
add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp std_msgs)
install(TARGETS simple_bag_recorder DESTINATION lib/\${PROJECT_NAME})
EOL
echo "CMakeLists.txt updated successfully."

# Step 3: Build the package
echo "Step 3: Building the package..."
cd ~/ros2_ws
colcon build --packages-select bag_recorder_nodes
echo "Package 'bag_recorder_nodes' built successfully."

# Step 4: Run the node
echo "Step 4: Running the node..."
source install/setup.bash
ros2 run bag_recorder_nodes simple_bag_recorder

