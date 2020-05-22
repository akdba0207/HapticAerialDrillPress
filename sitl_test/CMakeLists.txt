cmake_minimum_required(VERSION 2.8.3)
project(sitl_test)

#Author : Dongbin Kim
#Title : Simulation In The Loop test for Dongbin Kim's work.


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
)


 catkin_python_setup()


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sitl_test
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

#
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#SITL Practice
 add_executable(practice_armdisarm src/1_arm_disarm.cpp)
 add_executable(practice_phold src/2_position_hold.cpp)
 add_executable(practice_pcontrol src/3_position_control.cpp)

 #IROS 2020
 add_executable(iros2020_simdrillpress1 src/4_drillpress_position_control_iros2020.cpp)
 add_executable(iros2020_simdrillpress2 src/5_drillpress_position_control2_iros2020.cpp)

 ##Linking Libraries
 #SITL Practice
 target_link_libraries(practice_armdisarm
   ${catkin_LIBRARIES}
 )
 target_link_libraries(practice_phold
   ${catkin_LIBRARIES}
 )
 target_link_libraries(practice_pcontrol
 ${catkin_LIBRARIES}
)


#IROS 2020
target_link_libraries(iros2020_simdrillpress1
${catkin_LIBRARIES}
)
target_link_libraries(iros2020_simdrillpress2
${catkin_LIBRARIES}
)

