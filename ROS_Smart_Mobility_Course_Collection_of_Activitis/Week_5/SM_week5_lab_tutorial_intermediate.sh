#!/bin/sh

echo ---------- rosdep installation-----------
apt-get install python3-rosdep

echo ----------- rosdep operation ---------------
sudo rosdep init
rosdep update

# Installing dependencies
rosdep install --from-paths src -y --ignore-src

echo ------- creating an action ---------
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

echo ---- defining an action----------
cd action_tutorials_interfaces
mkdir action

echo --------- building an action-------------
# Create a file called Fibonacci.action with the action definition
cat > action/Fibonacci.action <<EOF
int32 order
---
int32[] sequence
---
int32[] partial_sequence
EOF

# Step 3: Build the action
# Update CMakeLists.txt and package.xml, then build the package

# Navigate back to the workspace's root directory
cd ../..

# Add rosidl code generation to CMakeLists.txt
# (Add these lines before the ament_package() line)
cat >> CMakeLists.txt <<EOF

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(\${PROJECT_NAME}
  "action/Fibonacci.action"
)
EOF

# Update package.xml with dependencies
# (Add these lines within the <buildtool_depend> and <depend> sections)
cat > package.xml <<EOF
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
EOF

echo " # Build the package "
colcon build

# Verify a successful build
# (Source the workspace and check the action definition)
source install/setup.bash
ros2 interface show action_tutorials_interfaces/action/Fibonacci

# Create the action_tutorials_cpp package
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

# Add visibility control to action_tutorials_cpp
echo "#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern \"C\"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility(\"default\")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility(\"default\")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility(\"hidden\")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_" > action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h

# Writing an action server
echo "#include <functional>
#include <memory>
#include <thread>

#include \"action_tutorials_interfaces/action/fibonacci.hpp\"
#include \"rclcpp/rclcpp.hpp\"
#include \"rclcpp_action/rclcpp_action.hpp\"
#include \"rclcpp_components/register_node_macro.hpp\"

#include \"action_tutorials_cpp/visibility_control.h\"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node(\"fibonacci_action_server\", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      \"fibonacci\",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), \"Received goal request with order %d\", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), \"Received request to cancel goal\");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), \"Executing goal\");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), \"Goal canceled\");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), \"Publish feedback\");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), \"Goal succeeded\");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
" > action_tutorials_cpp/src/fibonacci_action_server.cpp

# Writing an action client
echo "#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)"



# Start the component container
echo "Starting the component container..."
ros2 run rclcpp_components component_container &
container_pid=$!

# Wait for a moment to ensure the container is up
sleep 2

# Verify that the container is running
echo "Verifying the container is running..."
ros2 component list

# Load the talker component
echo "Loading the talker component..."
ros2 component load /ComponentManager composition composition::Talker

# Load the listener component
echo "Loading the listener component..."
ros2 component load /ComponentManager composition composition::Listener

# Verify the loaded components
echo "Verifying loaded components..."
ros2 component list

# Perform any other actions here as needed
# ...

# Optionally, unload components and stop the container
# echo "Unloading components..."
# ros2 component unload /ComponentManager 1 2

# Stop the component container
# echo "Stopping the component container..."
# kill -SIGINT $container_pid

# Optionally, run more demos as needed
# ...

# Exit the script
exit 0

