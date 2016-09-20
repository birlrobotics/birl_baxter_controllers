cmake_minimum_required(VERSION 2.8.3)
project(force_controller)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 sensor_msgs
 baxter_core_msgs
 kdl_parser
 dynamic_reconfigure
 message_generation
 genmsg
)

# ROS Independent packages
find_package(cmake_modules REQUIRED)
find_package(orocos_kdl)
find_package(Eigen REQUIRED)

## Dynamic Reconfigure: takes path to all cfg files taht we want to build.
generate_dynamic_reconfigure_options(cfg/force_error_constants.cfg)

# Generate message files in the 'msg' folder
add_message_files(DIRECTORY msg FILES setPoint.msg)

# Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES forceControl.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   baxter_core_msgs
   geometry_msgs
   sensor_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
	baxter_core_msgs
	sensor_msgs
	geometry_msgs
  dynamic_reconfigure
	message_runtime
)

include_directories( include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake} )

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Kinematics Library
add_library(mykinematics src/kinematics.cpp)
target_link_libraries(mykinematics ${catkin_LIBRARIES})

# 1a. Force Controller Service Server Executable (including a service server)
add_executable       (controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies     (controller %{PROJECT_NAME}_gencfg) # Builds dyn_rec headers before use.
add_dependencies     (mykinematics ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES} mykinematics) # links kin lib.

# 1b. Force Controller Service Client Executable
add_executable       (force_control_srv_client  src/force_control_srv_client/force_control_srv_client.cpp)
target_link_libraries(force_control_srv_client ${catkin_LIBRARIES})
add_dependencies     (force_control_srv_client %{PROJECT_NAME}_generate_messages_cpp)

# 2a. Force Controller Publisher Subscriber Executable (including a service server)
add_executable       (force_contrl_pubsub src/force_contr_pubsub.cpp)
target_link_libraries(force_contrl_pubsub ${catkin_LIBRARIES})
add_dependencies     (mykinematics ${catkin_EXPORTED_TARGETS})
add_dependencies     (force_contrl_pubsub %{PROJECT_NAME}_gencfg)    # Builds dyn_rec headers before use.
target_link_libraries(force_contrl_pubsub ${catkin_LIBRARIES} mykinematics) # lins kin lib

# 2b. Force Controller Set Point Publisher
add_executable       (setPoint_pub src/setPoint/setPoint_publisher.cpp)
target_link_libraries(setPoint_pub ${catkin_LIBRARIES})
add_dependencies     (setPoint_pub %{PROJECT_NAME}_generate_message_cpp)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
