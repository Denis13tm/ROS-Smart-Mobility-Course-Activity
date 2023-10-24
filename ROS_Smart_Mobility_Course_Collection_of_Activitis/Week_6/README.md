------------Enabling Topic Statistics -------------

**The README provides instructions on how to set up and run a ROS 2 C++ subscriber with topic statistics in the `cpp_pubsub` package. This allows you to enable topic statistics for a subscriber node, view the statistics data, and analyze the performance of your ROS 2 system.**

----- Prerequisites-----------

- ROS 2 installed on your system.
- A workspace with the `cpp_pubsub` package.

---------- Instructions-----------------

1. ---Navigate to the `cpp_pubsub` Package Directory:

   ```
   cd ~/ros2_ws/src/cpp_pubsub
   ```

    Download the Example Code with Topic Statistics:

    Download the example subscriber code with topic statistics:

    ```
    wget -O src/member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp


Modify CMakeLists.txt:

Add the new executable listener_with_topic_statistics to the CMakeLists.txt file. This allows you to build the subscriber with topic statistics:

```
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
```
Modify package.xml:

Add dependencies for rclcpp and std_msgs in the package.xml file:
```
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:build_depend" -t elem -v "rclcpp" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:build_depend" -t elem -v "std_msgs" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:exec_depend" -t elem -v "rclcpp" package.xml
xmlstarlet edit --inplace -N x="http://www.w3.org/1999/XML" -s "/x:package/x:exec_depend" -t elem -v "std_msgs" package.xml
```
Build the ROS 2 workspace:

Build your ROS 2 workspace to include the new listener_with_topic_statistics executable:
```
cd ~/ros2_ws
colcon build --symlink-install
```
Run the Subscriber with Topic Statistics:

Run the subscriber with topic statistics enabled:
```
ros2 run cpp_pubsub listener_with_topic_statistics
```
Run the Talker Node:

Open a new terminal and run the talker node (optionally, you can run this in the background):
```
ros2 run cpp_pubsub talker
```

View Available Topics:

In a new terminal, you can view the available topics:
```
ros2 topic list
```
Observe Published Statistics Data:

To observe the published statistics data, use the following command:
```
ros2 topic echo /statistics
```

The terminal will start displaying statistics messages every 10 seconds, as specified in the code.

# Using Fast DDS Discovery Server as discovery protocol
----------Running the Talker-Listener ROS 2 Demo-----------
The talker-listener ROS 2 demo consists of a talker node that publishes a "hello world" message every second and a listener node that subscribes to these messages. To set up and run this demo, follow the steps below.
---- Setup Discovery Server----------
Start by launching a discovery server with id 0, default port 11811, and listening on all available interfaces. Open a new terminal and run:
    
```
fastdds discovery --server-id 0
```

---------- Launch Listener Node-------------------
Execute the listener demo, which listens to the /chatter topic. In a new terminal, set the environment variable ROS_DISCOVERY_SERVER to the location of the discovery server. Ensure you've sourced ROS 2 in every new terminal:
```
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server
```
This command will create a ROS 2 node that automatically creates a client for the discovery server to perform discovery.

--------- Launch Talker Node-------------

Open a new terminal and set the ROS_DISCOVERY_SERVER environment variable as before to enable the node to start a discovery client:

```
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server
```

You should now see the talker node publishing "hello world" messages, and the listener node receiving these messages.

------------ Demonstrate Discovery Server Execution-------------
To demonstrate that the Discovery Server is working differently from the standard talker-listener example, run another node that is not connected to the discovery server. Start a new listener (listening on the /chatter topic by default) in a new terminal and check that it is not connected to the talker already running:
```
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener
```
The new listener node should not be receiving the "hello world" messages.

Finally, create a new talker using the simple discovery protocol (the default DDS distributed discovery mechanism) for discovery:
```
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker
```

Now, you should see the simple_listener node receiving "hello world" messages from simple_talker but not from talker_discovery_server.

---------- Visualization Tool rqt_graph------------

This example's nodes and structure can be verified using the rqt_graph tool. Set the ROS_DISCOVERY_SERVER environment variable before launching it to see the listener_discovery_server and talker_discovery_server nodes.
---------- Advanced Use Cases------------------

The sections that follow look at advanced capabilities of the Discovery Server that can be used to build more robust network setups and optimise ROS 2 introspection tools.
----------- Server Redundancy--------------

Multiple discovery servers can be setup with the fastdds tool. Discovery clients (ROS nodes) can connect to as many servers as they like, resulting in a redundant network that stays operational even if some servers or nodes fail unexpectedly.
---------- Backup Server---------------

The Fast DDS Discovery Server allows you to set up a server with backup capabilities. This enables the server to recover its previous state in the event of a shutdown, avoiding the requirement for a full rediscovery process and data loss.
------------- Discovery Partitions---------------------

Communication with discovery servers can be split to create virtual partitions in the discovery information. This means that two endpoints will only know about each other if there is a shared discovery server or a network of discovery servers between them.

-------------- ROS 2 Introspection------------------------

Introspection tools are supported by the ROS 2 Command Line Interface for analysing the behaviour of a ROS 2 network. Some utilities, such as ros2 bag record, ros2 topic list, and others, benefit from the capabilities of the Discovery Server. This section describes how to set up ROS 2 introspection tools to interact properly with the Discovery Server.
Daemon's Complementary Tools

Several ROS 2 CLI introspection tools make use of the ROS 2 Daemon. Configure the ROS 2 Daemon as a Super Client to use these tools using the Discovery Server technique.
There are no Daemon Tools.

The ROS 2 Daemon is not used by all ROS 2 CLI utilities. Instantiate these tools as Super Clients to link them to a Discovery Server and receive information from all topics.

----------- Compare Fast DDS Discovery Server with Simple Discovery Protocol

Scripts to generate network traffic traces and graphs are given for sophisticated users who want to compare nodes running with the Discovery Server against the default Simple Discovery Protocol. These scripts require that you have tshark installed on your machine.

Refer to the corresponding parts of the manual for detailed information on running these comparisons.

These advanced capabilities and situations demonstrate the ROS 2 Discovery Server's capability and flexibility for controlling and optimising ROS 2 network connection.


----------Implementing a custom memory allocator----------------------
                               &&&&&&&&
------------- Writing the Custom Allocator---------------

Initiate with creating a custom allocator. Here's a basic example of what the structure of a custom allocator might look like:
```
template <class T>
struct custom_allocator {
  using value_type = T;

  custom_allocator() noexcept;
  template <class U> custom_allocator(const custom_allocator<U>&) noexcept;

  T* allocate(std::size_t n);
  void deallocate(T* p, std::size_t n);
};

template <class T, class U>
constexpr bool operator==(const custom_allocator<T>&, const custom_allocator<U>&) noexcept;

template <class T, class U>
constexpr bool operator!=(const custom_allocator<T>&, const custom_allocator<U>&) noexcept;
```
Use functions like allocation and deallocate to implement your custom allocator logic. Customise it to meet your individual memory management requirements.

Save your custom allocator code in a .cpp or .h file, depending on your project structure.

------ Usage

Now that you've implemented your custom allocator, you can use it in your ROS 2 project. Here's an example of how to use it:

Include your custom allocator header in your ROS 2 C++ source files.

Instantiate your custom allocator and use it for memory allocation and deallocation, similar to how you would use the standard allocator:
```
#include "custom_allocator.h"

int main() {
  // Instantiate your custom allocator
  custom_allocator<int> my_allocator;

  // Use your allocator to allocate memory
  int* my_memory = my_allocator.allocate(5);

  // Use the allocated memory

  // Deallocate the memory when done
  my_allocator.deallocate(my_memory, 5);

  return 0;
}
```
----------Testing------------------

To test your custom allocator, follow these steps:

Compile your ROS 2 project, including the custom allocator code.

Run your ROS 2 application. During execution, your custom allocator will be used for memory allocation and deallocation.

Monitor the behavior of your custom allocator to ensure it meets your memory management requirements.

---------- Passing an Allocator to the Intra-Process Pipeline

If you need to use your custom allocator within the ROS 2 intra-process communication pipeline, you can pass it to your ROS nodes and components as demonstrated in the code examples in your original documentation.

---------- Additional Notes

Some compilers with partial C++11 support may require additional boilerplate code when working with standard library structures. Adjust your allocator implementation accordingly if needed.

Be aware that certain memory allocations and deallocations may originate in the underlying DDS (Data Distribution Service) implementation used by ROS 2.

Now you have the knowledge to create and use a custom allocator compatible with ROS 2. Customize your allocator to meet the specific memory management needs of your ROS 2 projects.

------ Unlocking the potential of Fast DDS middleware [community-contributed]
------------Mixing Synchronous and Asynchronous Publications in the Same Node
In this example, we demonstrate how to create a ROS 2 node with two publishers, one using synchronous publication mode, and the other using asynchronous publication mode.
By default, rmw_fastrtps uses synchronous publication mode.
--------------Synchronous Publication Mode
In synchronous publication mode, data is sent directly within the context of the user thread. This means that any blocking call during the write operation will block the user thread, potentially preventing the application from continuing its operation. However, this mode typically offers higher throughput rates and lower latencies since there's no notification or context switching between threads.
------------- Asynchronous Publication Mode
In asynchronous publication mode, when the publisher invokes the write operation, the data is copied into a queue. A background thread (asynchronous thread) is notified about the addition to the queue, and control of the thread is returned to the user before the data is actually sent. The background thread is responsible for consuming the queue and sending the data to every matched reader.

Let's create a ROS 2 node with both synchronous and asynchronous publishers.
--------- Create the Node with the Publishers-------------
----------- 1. Create a New Package-------------
First, create a new package named sync_async_node_example_cpp within a new workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp
```
------------- 2. Add Source File for the Node -------------------
Add a file named src/sync_async_writer.cpp to the package with the following content:
```
// Include necessary headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SyncAsyncPublisher : public rclcpp::Node
{
public:
    SyncAsyncPublisher()
        : Node("sync_async_publisher"), count_(0)
    {
        // Create the synchronous publisher on topic 'sync_topic'
        sync_publisher_ = this->create_publisher<std_msgs::msg::String>("sync_topic", 10);

        // Create the asynchronous publisher on topic 'async_topic'
        async_publisher_ = this->create_publisher<std_msgs::msg::String>("async_topic", 10);

        // This timer will trigger the publication of new data every half a second
        timer_ = this->create_wall_timer(
                500ms, std::bind(&SyncAsyncPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Create a new message to be sent
        auto sync_message = std_msgs::msg::String();
        sync_message.data = "SYNC: Hello, world! " + std::to_string(count_);

        // Log the message to the console to show progress
        RCLCPP_INFO(this->get_logger(), "Synchronously publishing: '%s'", sync_message.data.c_str());

        // Publish the message using the synchronous publisher
        sync_publisher_->publish(sync_message);

        // Create a new message to be sent
        auto async_message = std_msgs::msg::String();
        async_message.data = "ASYNC: Hello, world! " + std::to_string(count_);

        // Log the message to the console to show progress
        RCLCPP_INFO(this->get_logger(), "Asynchronously publishing: '%s'", async_message.data.c_str());

        // Publish the message using the asynchronous publisher
        async_publisher_->publish(async_message);

        // Prepare the count for the next message
        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr async_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncAsyncPublisher>());
    rclcpp::shutdown();
    return 0;
}
```
--------------3. Modify CMakeLists.txt------------
Open the CMakeLists.txt file and add the new executable for SyncAsyncWriter so you can run the node using ros2 run:
```
add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

install(TARGETS
    SyncAsyncWriter
    DESTINATION lib/${PROJECT_NAME})
```
Clean up the CMakeLists.txt file by removing unnecessary sections and comments, so it looks like this:
```
cmake_minimum_required(VERSION 3.8)
project(sync_async_node_example_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

install(TARGETS
    SyncAsyncWriter
    DESTINATION lib/${PROJECT_NAME})

ament_package()
```
------------ 4. Build the Package -------------------
Now build the package:
```
cd ~/ros2_ws
colcon build --packages-select sync_async_node_example_cpp
```
This concludes the setup for the node with publishers.
--------------- Create the XML Configuration File
Create a file named SyncAsync.xml with the following content:
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- Default publisher profile -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </publisher>

    <!-- Default subscriber profile -->
    <subscriber profile_name="default_subscriber" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </subscriber>

    <!-- Publisher profile for topic sync_topic -->
    <publisher profile_name="/sync_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>SYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

    <!-- Publisher profile for topic async_topic -->
    <publisher profile_name="/async_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

 </profiles>

This XML configuration file defines several profiles for publishers and subscribers, including profiles for both synchronous and asynchronous publishers for the topics sync_topic and async_topic.
------ Execute the Publisher Node
Before running the publisher node, export the required environment variables for the XML configuration to be loaded:
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml
```
----Execute the subscriber node
Export the required environment variables for the XML to be loaded:


---- Using other FastDDS capabilities with XML---
----- Limiting the number of matching subscribers---
Add a maximum number of matched subscribers to the /async_topic publisher profile. It should look like this:


You should see that only the first subscriber node receives the messages from both topics. The second one could not complete the matching process in the /async_topic because the publisher prevented it, as it had already reached its maximum of matched publishers. Consequently, only the messages from the /sync_topic are going to be received in this third terminal:

------- Using partitions within the topic------------
Let us change the /sync_topic publisher to partition part1 and create a new /sync_topic subscriber that uses the partition part 2. Their profiles should now look like this:

The /sync_topic subscriber is not receiving the data as it is in a different partition from the corresponding publisher.

-------- Recording a bag from a node (C++) ---------
Before starting this tutorial, make sure you have the following prerequisites:
ROS 2 installed as part of your regular ROS 2 setup.
If you've installed ROS 2 from Debian packages on Linux, the rosbag2 tool may be installed by default. If not, you can install it using the following command:
```
sudo apt install ros-humble-rosbag2
```
---- Create a ROS2 Package---------
In this task, you will create a ROS 2 package called bag_recorder_nodes that contains nodes for recording and playing back data. Follow these steps:
```
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs
```
---- Write a C++ Node----------------
In this task, you will write a C++ node called simple_bag_recorder that subscribes to a topic, receives data, and saves it to a bag file. Follow these steps:

Inside the ros2_ws/src/bag_recorder_nodes/src directory, create a new file called simple_bag_recorder.cpp and paste the provided C++ code into it.

The code in simple_bag_recorder.cpp sets up a node that subscribes to a topic, receives data, and writes it to a bag file. The code is well documented to explain its functionality.

Pay attention to the dependencies included at the top of the file. These are necessary for interacting with ROS 2 and working with bag files.

The class SimpleBagRecorder represents the node. It sets up a writer for the bag file, subscribes to the "chatter" topic, and defines a callback for incoming data.

The main function initializes the ROS 2 context, spins the node, and shuts down when finished.
 Update package.xml

You don't need to manually add dependencies to package.xml or CMakeLists.txt because the --dependencies argument was used during package creation. However, make sure to update the package description, maintainer email and name, and license information in package.xml:

-------Update package.xml-----------------
You don't need to manually add dependencies to package.xml or CMakeLists.txt because the --dependencies argument was used during package creation. However, make sure to update the package description, maintainer email and name, and license information in package.xml:




