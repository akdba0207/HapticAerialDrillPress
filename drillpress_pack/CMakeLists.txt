cmake_minimum_required(VERSION 2.8.3)
project(drillpress_pack)

#Author : Dongbin Kim
#Title :  Haptic Drill press pacakge for IROS 2020
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dxl_cpp_test
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
  /home/dbkim-ros/DynamixelSDK/c++/include/dynamixel_sdk
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(Drillpress_node src/Drillpress_pubsub.cpp)
 add_executable(Gantry_Torque src/Torque_read_pub.cpp)
 add_executable(Gantry_node src/GantryControl.cpp)
 add_executable(Data_node src/Data_arrangement.cpp)
 add_executable(UAV_drillsensor src/UAV_Drillsensor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(Drillpress_node
  ${catkin_LIBRARIES}
  dxl_x64_cpp
)

target_link_libraries(Gantry_Torque
  ${catkin_LIBRARIES}
  dxl_x64_cpp
)

target_link_libraries(Gantry_node
  ${catkin_LIBRARIES}
  dxl_x64_cpp
)

target_link_libraries(UAV_drillsensor
  ${catkin_LIBRARIES}
  dxl_x64_cpp
)

target_link_libraries(Data_node
  ${catkin_LIBRARIES}
)


