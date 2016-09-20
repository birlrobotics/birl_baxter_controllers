cmake_minimum_required(VERSION 2.8.3)
project(position_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
	actionlib
	actionlib_msgs
	control_msgs
	trajectory_msgs
	baxter_core_msgs
	geometry_msgs 
	roscpp 
	rospy 
	sensor_msgs 
	std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
	actionlib
	actionlib_msgs
	control_msgs
	trajectory_msgs
	baxter_core_msgs
	geometry_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(position src/initialPose.cpp)

## Declare a cpp library
add_library(iniPose src/initialPose.cpp)

target_link_libraries(position ${catkin_LIBRARIES})


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
