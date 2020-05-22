cmake_minimum_required(VERSION 2.8.3)
project(flight_test)

#Author : Dongbin Kim
#Title : MM-UAV flight test for Dongbin Kim's work.

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flight_test
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_phold src/ftest_poshold.cpp)

target_link_libraries(${PROJECT_NAME}_phold
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_pcmd src/ftest_poscommand.cpp)

target_link_libraries(${PROJECT_NAME}_pcmd
  ${catkin_LIBRARIES}
)


#Haptic based MM-UAV to drill
add_executable(${PROJECT_NAME}_pid_drill src/ftest_pidcontrol2_drill.cpp)

target_link_libraries(${PROJECT_NAME}_pid_drill
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_setpoint_drill src/ftest_setpoint2_drill.cpp)

target_link_libraries(${PROJECT_NAME}_setpoint_drill
  ${catkin_LIBRARIES}
)