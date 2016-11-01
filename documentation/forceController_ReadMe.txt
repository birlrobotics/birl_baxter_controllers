[Overview]
This controller is an implementation of the control basis framework [1,2]. It works under the assumption one, two, or more controllers can concurrently work together and optimize their goals. The typical case is for two controllers to run together. I.e. a force controller works as the dominant controller and a moment controller works as the subordiante controller. The force controller serves as an outer loop that outputs the change in joint angles to an inner joint controller. 
The force controller is found under birlBaxter_controllers/force_controller
The position controller is found under birlBaxter_controllers/position_controller

[Implementations]
Two implementations have been made for the same code. One using a ROS service to pass the setpoint and one using a ROS topic. The name of the .cpp are given below:
ROS Service: force_controller/src/controller.cpp
ROS Topic:   force_controller/src/force_contr_pubsub.cpp

[Inputs]
The set point for the system is provided by the setPoint_publisher.cpp. This programs takes a varying number of arguments that can be set manually or through a launch file including, number of controllers, the type of each sub-controller, the set-point and equivalent gains.
//--------------------------------------------------------------------------------------------------- 
// Command line arguments can be expected in any of the following forms:
// numCtrls mode1 desx1 desy1 desz1                                                   # argc= 6 => 1 controller with setpoint 
// numCtrls mode1 desx1 desy1 desz1 gx1 gy1 gz1                                       # argc= 9 => 1 controller with setpoint and mode
//--------------------------------------------------------------------------------------------------- 
// numCtrls mode1 desx1 desy1 desz1 mode2 desx2 desy2 desz2                           # argc=10 => 2 controller with setpoints
// numCtrls mode1 desx1 desy1 desz1 gx1 gx2 gx3 mode2 desx2 desy2 desz2               # argc=13 => 2 controller 1st one with setpoints and gains, 2nd one only with setpoint
// numCtrls mode1 desx1 desy1 desz1 gx1 gx2 gx3 mode2 desx2 desy2 desz2 gx2 gy2 gz2   # argc=16 => 2 controller with setpoints and gains
//--------------------------------------------------------------------------------------------------- 

by manually publishing a message to either the "/right/force_control/setPoint" or "/left/force_control/setPoint" topic. The type of the messager is: force_controller/setPoint, which is defined in birlBaxter_controllers/force_controller/msg. This message requires:

1. Give the total number of controllers, i.e. 1 or 2.
2. Give the name of the domType of controller, i.e.: "force" | out of force/moment.
3. Give the desired set point for the dominant controller, i.e.: x=0, y=0, z=0. 
4. Give the gains (if customized) for the dominang Gain. 

The same process repeats if there is a second controller.
5. Name of subordinate controller: i.e. "moment"
6. Desired setpoint for subordinate controller.
7. Desired gains for subordiante controllers.

Note:
If no parameter information is given, the code will use standard parameter values.

[Launch Files and Running]
Three launch files: one to publish the setpoint and the other two, to choose what kind of communication to use for the controller, either a service/topic:

SetPoint:
	setPoint_publisher.launch
Controller:
	Service: right/left_client_controller.launch
	Topic:   right/left_force_controller_topic.launch

FT Sensor:
	you will also need to:
  - ensure that the robot is using the right URDF or adapt the mass at end-effector with a gripper customization script.
  - run the ft_wacoh ft_wacoh_pub_2 node to run ft_sensor. 

[Pseudo-code]
- in main:
	- create node handle
	- instantiate node
	- perform dynamic reconfigure
	- call force controller << can be done with different communication modes: 
	  spinOnce, AsyncSpinner, MultiThreadSpinner >>

- Constructor:
	- initializes all topics (publish/subscribe) as well as all services & the kinematic chain. 
	  // One spinner per ROS communication object: here we use it for 
          // 1. Publish joint commands
          // 2. Subsribe to current joint Angles
          // 3. Advertice a service server/topic
          // 4. Subscribe to endpoint wrench (optional set in getWrenchEndpoint)
          // 5. Dynamic Reconfigure 
          // 6. published filtered wrench

- get wrench set point:
controller::getWrenchEndpoint
