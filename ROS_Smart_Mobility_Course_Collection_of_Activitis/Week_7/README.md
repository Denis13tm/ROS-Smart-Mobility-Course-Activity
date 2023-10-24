# Image Processing(Edge Detection)
## Application Description
## Project Overview
The project aims to demonstrate a ROS 2 application for performing computer vision tasks using an action server-
client paradigm. The action server will perform a simple image processing task, such as edge detection, and the
client will request the server to process an image.
### Nodes
1. SM_action_image_processor_server.cpp: Action server node that processes images and performs edge
detection.
2. SM_image_processor_client.cpp: Action client node that sends image processing requests to the server.
# Interaction Diagram:
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/f2facc10-abb6-4ebf-b4e4-6c7ffb13a47a)
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
In the CMakeLists.txt file, we define the dependencies on ROS 2 packages, create two executables (for the server and client), and specify where to install the executables.

In the package.xml file, we specify the package name, version, and dependencies for building and running the project. You should replace your_email@example.com and Your Name with your actual contact information.
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
Feel free to expand on this README with more specific details and guidelines to suit your project's needs.
