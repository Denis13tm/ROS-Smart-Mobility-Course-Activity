cmake_minimum_required(VERSION 3.5)
project(ros2_cv_action_project)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

# Add the server and client executables
add_executable(image_processor_server src/image_processor_server.cpp)
ament_target_dependencies(image_processor_server rclcpp rclcpp_action example_interfaces)

add_executable(image_processor_client src/image_processor_client.cpp)
ament_target_dependencies(image_processor_client rclcpp rclcpp_action example_interfaces)

# Install executables
install(TARGETS
  image_processor_server
  image_processor_client
  DESTINATION lib/${PROJECT_NAME})

# Create ament index resource which references the libraries in the binary directory
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

