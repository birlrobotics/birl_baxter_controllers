This force controller can be run in two fashions:

1) Using topics, 
2) Using a service

----------------------------------------
A. How to start controller using topics
----------------------------------------

FILES
Set Point: topic that publishes desired forces/moments:
./src/setPoint/setPoint_publisher.cpp
./launch/setPoint_publisher/setPoint_publisher.launch

Force/Moment Controller:
./src/force_controller_topic.cpp
./include/force_controller/force_controller_topic.h
./launch/topic_controller/"side"_force_controller_topic.launch

HOW TO LAUNCH
- Start the force controller
roslaunch force_controller right_force_controller_topic.launch

- Start the setpoint
roslaunch force_controller setPoint_publisher.launch


PARAMETERS TO MODIFY
- The main parameters for modification will be found in the setPoint_publisher.launch file. Here you can set:
- How many controllers do you want to run: 1 or 2 (Dec 2017)
- Which controller is dominant (0=force controlller, 1=moment controller)?
- What is your setpoint (Fxyz or Mxyz)
- What are your gains?

Otherwise hardcoded values are set in:
./include/force_controller/force_controller_topic.h

-------------------------------------------
B. How to start controller using a service
-------------------------------------------

FILES

Force/Moment Controller:
./src/force_control_srv_client/force_control_srv_client.cpp
./src/force_controller_service.cpp
./include/force_controller/force_controller_service.h

HOW TO LAUNCH
- Start the server
roslaunch force_controller right_server_controller.launch

- Start the client
roslaunch force_controller right_force_controller_topic.launch

OR 

- Manually for right (change side for left):
rosservice call /right/force_controller "{num_ctrls: 1, type: [force], desired: [{x: 1.0, y: 0.0, z: 0.0}], gains: []}"


PARAMETERS TO MODIFY
- The main parameters for modification will be found in the setPoint_publisher.launch file. Here you can set:
- How many controllers do you want to run: 1 or 2 (Dec 2017)
- Which controller is dominant (0=force controlller, 1=moment controller)?
- What is your setpoint (Fxyz or Mxyz)
- What are your gains?

Otherwise hardcoded values are set in:
./include/force_controller/force_controller_topic.h
