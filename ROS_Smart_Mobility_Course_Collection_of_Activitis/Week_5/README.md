# ROS2 Tutorial: Intermediate
## Managing Dependencies with rosdep
### rosdep installation
If you are using rosdep within the ROS ecosystem, it is conveniently included with the ROS distribution, which is the recommended method for obtaining rosdep. You can install it by running:

```

apt-get install python3-rosdep
```
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/b8f55a7e-ba99-428b-a0a9-621fb5f30c20)

Note: For Debian and Ubuntu users, please ensure that you do not have a similarly named package called python3-rosdep2 installed before proceeding with the installation of python3-rosdep.

If you are using rosdep outside of ROS or the system package is unavailable, you can install it directly from PyPI using pip:

```bash

pip install rosdep
```

## ROSdep Operation

Now that you have rosdep installed, you can use it to manage dependencies. If you are new to rosdep, make sure to initialize it with the following commands:

```bash

sudo rosdep init
rosdep update
```
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/642bc5c3-aaf8-415e-b57d-7f73c35e2250)

This initializes rosdep and updates the locally cached rosdistro index. Periodically updating rosdep is advisable to ensure you have the latest index.

To install dependencies, you can use rosdep install. Typically, you would run this command over a workspace containing multiple packages to install all dependencies. Here's an example command when in the root of the workspace with a 'src' directory that contains source code:

```bash

rosdep install --from-paths src -y --ignore-src
```
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/e4839ed9-ef0c-4e1c-94d8-ddad2bf9a333)

Breaking down the command:

    --from-paths src specifies the path to check for package.xml files to resolve keys.
    -y answers 'yes' to all prompts from the package manager, enabling unattended installation.
    --ignore-src instructs rosdep to ignore installing dependencies if the package itself is in the workspace.

Feel free to customize the rosdep install command as needed for your specific workspace and dependencies.

## Creating an action
### Creating a package
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/48f6ce8b-c06e-4a5a-9ed9-bdadb98b15c5)

### Defining an Action

In ROS 2, actions are defined using .action files, which consist of three message definitions separated by ---:

Request: The request message is sent from an action client to an action server, initiating a new goal.
Result: The result message is sent from an action server to an action client when the goal is completed.
Feedback: Feedback messages are periodically sent from an action server to an action client, providing updates about the goal's progress.


    
Example: Fibonacci Action

Let's define a new action named "Fibonacci" for computing the Fibonacci sequence. To do this, follow these steps:

Create an action directory within your ROS 2 package named action_tutorials_interfaces.
```
cd action_tutorials_interfaces
mkdir action
```
Within the action directory, create a file named Fibonacci.action with the following contents:

markdown

    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence

        The request (order) specifies the order of the Fibonacci sequence to compute.
        The result (sequence) is the final computed sequence.
        The feedback (partial_sequence) provides the partial sequence computed so far.

A single instance of an action is commonly referred to as a "goal."
### Building an Action

Before you can use the new Fibonacci action in your code, you must pass the definition to the rosidl code generation pipeline. Follow these steps to build the action:  

Add the following lines to your CMakeLists.txt file before the ament_package() line, inside the action_tutorials_interfaces package:

   
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
Make sure to add the required dependencies to your package.xml:

```

<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
Note: You need to depend on action_msgs because action definitions include additional metadata, such as goal IDs.

Build the package containing the Fibonacci action definition:

```

    # Change to the root of the workspace
    cd ~/ros2_ws
    # Build
    colcon build
```
### Verification

By convention, action types are prefixed by their package name and the word "action." In our case, when referring to the new action, use the full name action_tutorials_interfaces/action/Fibonacci.

You can check that the action was built successfully using the command-line tool:

```

# Source your workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that the action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/6298c001-7370-4d54-9a47-65b05f2a5931)

With these steps, you have defined and built a custom ROS 2 action for computing the Fibonacci sequence. You can now use this action in your ROS 2 projects.

## Writing an action server and client (C++)
### Creating the action_tutorials_cpp package

To create the action_tutorials_cpp package for the C++ action server and client, follow these steps:

```

cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
```
This command will create a new package with the necessary dependencies.
### Writing an action server

In this step, we'll focus on writing the action server. The action server will calculate the Fibonacci sequence based on the action we created earlier. Follow these steps:

Open the file action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h and add the provided visibility control code. This step ensures proper visibility control for Windows.

Create the action server code in the file action_tutorials_cpp/src/fibonacci_action_server.cpp using the provided code.

### Writing an action client

In this step, we'll write the action client. The action client will send goal requests to the action server to calculate the Fibonacci sequence. Follow these steps:

Create the action client code in the file action_tutorials_cpp/src/fibonacci_action_client.cpp using the provided code.

Building and Running

To compile and run the action server and client, follow these steps:

Build the ROS 2 workspace:

```

cd ~/ros2_ws
colcon build
```
Start the action server in a separate terminal:

```

source ~/ros2_ws/install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_server
```
Open another terminal and run the action client:

```

source ~/ros2_ws/install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_client
```
You should see logged messages indicating the goal being accepted, the feedback being printed, and the final result.

## Composing multiple nodes in a single process
### Discover Available Components

Objective: View a list of registered components in your workspace.

Instructions:

Open a terminal.

Run the following command to see a list of available components:

    ros2 component types

The terminal will display a list of available components from various packages.

### Run-time Composition with Publisher and Subscriber

Objective: Demonstrate run-time composition using ROS services with a publisher and subscriber.

Instructions:
Open two terminals.

In the first terminal, start the component container:
```
ros2 run rclcpp_components component_container
```
In the second terminal, verify that the container is running:
```
ros2 component list
```
You should see a component named /ComponentManager.

In the second terminal, load the talker component:

```
ros2 component load /ComponentManager composition composition::Talker
```
The command will return a unique ID for the loaded component and its node name.

The first terminal should now display messages indicating that the component was loaded and is publishing messages.

In the second terminal, load the listener component:

```

    ros2 component load /ComponentManager composition composition::Listener
```
Terminal will return information about the loaded component.

Use the ros2 component list command in the second terminal to verify that both components are loaded.

The first terminal should now display messages indicating that the listener is receiving messages from the talker.

### Run-time Composition with Server and Client

Objective: Demonstrate run-time composition using ROS services with a server and client.

Instructions:

Open two terminals.

In the first terminal, start the component container:

```
ros2 run rclcpp_components component_container
```
In the second terminal, load the server and client components:

```

    ros2 component load /ComponentManager composition composition::Server
    ros2 component load /ComponentManager composition composition::Client
```
In this scenario, the client sends a request to the server, and the server processes the request and replies with a response. The client terminal will display the received response.

### Compile-time Composition using ROS Services

Objective: Demonstrate compile-time composition using ROS services.

Instructions:

Open a terminal.

Run the following command to run the demo:

```

    ros2 run composition manual_composition
```
This will show repeated messages from both pairs: the talker and the listener, as well as the server and the client.

Note: Manually-composed components will not be reflected in the ros2 component list command line tool output.

### Run-time Composition using dlopen

Objective: Demonstrate run-time composition using dlopen.

Instructions:

Open a terminal.

Run the following command to execute the demo:
```
    ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```
The terminal should display repeated output for each sent and received message.

Note: dlopen-composed components will not be reflected in the ros2 component list command line tool output.

### Composition using Launch Actions

Objective: Demonstrate composition using launch actions.

Instructions:

Open a terminal.

Run the following command to use a launch file for composition:
```
ros2 launch composition composition_demo.launch.py
```
