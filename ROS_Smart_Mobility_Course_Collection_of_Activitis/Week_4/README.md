# ROS2

## Using turtlesim, ros2, and rqt
### Starting turtlesim
Starting the turtlesim simulator.
-

### Using turtlesim
Running a node to control the turtle in the simulator.
![image](https://github.com/Denis13tm/ROS-Smart-Mobility-Course-Activity/assets/69725678/d2fa8a05-64f3-45ee-b6df-c448f6a34a35)

### Using rqt
rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.
![image](https://github.com/Denis13tm/ROS-Smart-Mobility-Course-Activity/assets/69725678/06d52545-dbd3-4e21-8b88-967543e70714)

## Trying the spawn service
/spawn will create another turtle in the turtlesim window.
-
### Trying the set_pen service
The values for r, g, and b, which are between 0 and 255, set the color of the pen turtle1 draws with, and width sets the thickness of the line.

### Remapping
Remaping the teleoperation node to control different turtles.

## Understanding nodes
### ros2 node list
Listing nodes in the ROS2 system.

### Remapping 
Reassigning the name of our /turtlesim node.

### ros2 node info
Accessing more information about nodes.

## Understanding topics
### rqt_graph
Visualizing the changing nodes, topics, and the connections between them.

### ros2 topic list
Listing all the topics currently active in the system.

### ros2 topic echo
Seeing the data being published on a topic.

### ros2 topic info
Looking topics.

### ros2 interface show
Learning details of messages.

### ros2 topic pub
Publishing data onto a topic directly

## Understanding services
### ros2 service list
Listing all the services currently active in the system.

### ros2 service type
Finding out the type of services.

### ros2 service list -t

### ros2 service find 
Finding services of a specific type.


### ros2 interface show 


### ros2 service call
Calling a service.


## Understanding parameters
### ros2 param list

### ros2 param get

### ros2 param set

### ros2 param dump


### ros2 param load

### Load parameter file on node startup

## Understanding actions
### Use actions

### ros2 node info

### ros2 action list

### ros2 action list -t

### ros2 action info

### ros2 interface show

### ros2 action send_goal

## Using rqt_console to view logs
### Setup

### Messages on rqt_console
### Set the default logger level

## Launching nodes
### Running a Launch File


### Control the Turtlesim Nodes

## Recording and playing back data
### Choose a topic

### ros2 bag record

### Record multiple topics

### ros2 bag info 

### ros2 bag play









