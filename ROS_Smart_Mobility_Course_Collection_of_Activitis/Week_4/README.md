# ROS2

## Using turtlesim, ros2, and rqt
### Starting turtlesim
Starting the turtlesim simulator.

![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/9f59f63c-d743-43d5-86fd-4d83d24a516f)
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/afc19565-fabb-4ed2-9301-13bb772724a9)

### Using turtlesim
Running a node to control the turtle in the simulator.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/7892ecd1-2006-4994-bf34-dcecc2a11632)

### Using rqt
rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/06a68a95-0d85-46f1-99fb-b59ad2a0733d)

### Trying the spawn service
/spawn will create another turtle in the turtlesim window.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/4288e874-69be-4ff3-8737-560e05148cf3)

### Trying the set_pen service
The values for r, g, and b, which are between 0 and 255, set the color of the pen turtle1 draws with, and width sets the thickness of the line.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/a8ffd85f-e9e4-47ba-8fb7-d2e988893145)

### Remapping
Remaping the teleoperation node to control different turtles.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/39f6a7ab-f079-4306-b73d-414d9d488d39)

## Understanding nodes
### ros2 node list
Listing nodes in the ROS2 system.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/b3d629f5-37a9-4f07-8674-c35f3150b885)
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/584ca53e-85e4-4a18-94f3-eede33927888)

### Remapping 
Reassigning the name of our /turtlesim node.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/b3eb342b-0493-4923-9760-3f54de280eca)

### ros2 node info
Accessing more information about nodes.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/00af2822-c73c-467b-a22e-363d691fe814)

## Understanding topics
### rqt_graph
Visualizing the changing nodes, topics, and the connections between them.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/38f7a0ce-6ce4-457a-bc03-54e6c5eab2c3)

### ros2 topic list
Listing all the topics currently active in the system.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/20537c2d-b6cb-4ea3-8b8f-0410ddda2842)

### ros2 topic echo
Seeing the data being published on a topic.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/b8a339c4-d8e0-435d-970e-89e6df51acd0)

### ros2 topic info
Looking topics.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/2a6872d3-99b8-40a9-89a3-a166b246aa41)

### ros2 interface show
Learning details of messages.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/fef5eec9-afb5-4676-8aa9-5eae85885a21)

### ros2 topic pub
Publishing data onto a topic directly
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/ee78aeed-7a6a-488f-99c8-f40863b18ae7)

## Understanding services
### ros2 service list
Listing all the services currently active in the system.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/13bc0357-abbf-4226-bcd5-9b7e698eb59a)

### ros2 service type
Finding out the type of services.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/65dd5d81-abe7-4e4b-af6a-e4879b467cb8)

### ros2 service list -t
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/efe74636-f4d9-41a9-8def-348dd40c5ce2)

### ros2 service find 
Finding services of a specific type.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/ff035d47-cde3-4efe-b47c-3599ff86a84f)

### ros2 interface show 
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/ccd12afc-a9d9-482b-b5f9-93a9ce0cd6d1)

### ros2 service call
Calling a service.
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/96cf4b78-b08b-4501-abfe-04824136b152)

## Understanding parameters
### ros2 param list

![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/3199f87d-a033-4ebe-9ca0-565b12a80347)

### ros2 param get
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/85405964-abda-4789-aa8e-273545fef7f1)

### ros2 param set
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/11254d96-449f-48e8-9b8c-ce7ea1030c31)

### ros2 param dump
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/6b640548-f5df-4c86-ad2f-ac1805b049fa)

### ros2 param load
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/78982129-f8e6-4391-b58b-80905581caa6)

### Load parameter file on node startup
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/1d28e93e-d47c-42e6-916f-c8a010b693d4)

## Understanding actions
### Use actions
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/958c60dd-f221-4980-9d7f-0fa9f8c457a8)

### ros2 node info
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/df9e0dd3-a257-44fa-a356-42851ef5ef9e)

### ros2 action list
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/37cd2268-2b4b-4453-92ee-aea6e537c155)

### ros2 action list -t
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/68410d51-772e-4e6a-acea-869f88c413c3)

### ros2 action info
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/79c1a7f4-6907-4e13-9bb6-e880d6b3e6af)

### ros2 interface show
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/8f169e8a-6e8c-4a88-9081-f6556ac9b84b)

### ros2 action send_goal
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/15914026-114b-40a8-a51e-1c78b3012272)
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/b1f42a2d-207a-46ed-b51a-71f58478ed1c)

## Using rqt_console to view logs
### Setup
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/64223a4b-c214-4bbc-b287-b44429b8535a)

### Messages on rqt_console
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/81f89c2c-8feb-4a8d-ac2f-999046ea0c7d)

### Set the default logger level
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/5ae83b97-5b19-44c1-849c-5e4890c9c60f)

## Launching nodes
### Running a Launch File
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/bf712c39-df2f-49c1-a5bb-9d1f1a9f6f62)

### Control the Turtlesim Nodes
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/3e5259e2-402c-4524-a74e-dd8efe59e293)

## Recording and playing back data
### Choose a topic
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/25076fc9-598f-47c3-9918-a0b75efd909f)

### ros2 bag record
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/3c3ae3a6-27df-4f87-8b07-86198a874b41)

### Record multiple topics
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/32c974bd-ddad-4ccd-b9d7-b546bb9cc53c)

### ros2 bag info 
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/5a4e31d2-0f6e-426a-8827-1e38fc87fd90)

### ros2 bag play
![image](https://github.com/ImAli0/ROS_Smart_Mobility_Course_activities/assets/113502495/a9cf628e-fdd3-403b-b3bd-8204bf4741e5)









