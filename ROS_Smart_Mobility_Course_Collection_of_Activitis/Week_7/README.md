# Image Processing(Edge Detection)
## Application Description
## Project Overview
The goal of this project is to present a ROS 2 application that performs computer vision tasks utilising an action server-client architecture. The action server will do a simple image processing task, such as edge detection, while the client requests that the server process an image.

### Nodes
1. SM_action_image_processor_server.cpp: Action server node for image processing and edge detection.
2. SM_image_processor_client.cpp: Action client node responsible for sending image processing requests to the server.
# Diagram of Interaction:

# Interaction Diagram:


## File Structure
```
ros2_cv_action_project/
│-- src/
│   |-- image_processor_server.cpp
│   |-- image_processor_client.cpp
│-- CMakeLists.txt
│-- package.xml
│-- README.md
```
It declares the dependencies on ROS 2 packages in the CMakeLists.txt file, produces two executables (for the server and client) and specifies where to install the executables.

The package name, version, and dependencies for building and running the project are specified in the package.xml file. EMAIL(otabektuychievofficial@gmail.com) and NAME(Otabek) should be replaced with your actual contact information.

# Build the Project
```
colcon build --packages-select ros2_cv_action_project
```
# Running the Project
### Starting the Action Server
```
ros2 run ros2_cv_action_project image_processor_server
```
### Sending a request with the action client
```
ros2 run ros2_cv_action_project image_processor_client
```
End.
