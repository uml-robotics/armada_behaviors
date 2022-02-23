cmake_minimum_required(VERSION 3.0.2)
project(armada_flexbe_utilities)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  actionlib_msgs
  moveit_core
  #moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES CartesianMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES armada_flexbe_utilities
  CATKIN_DEPENDS
  actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(armada_manipulation_server src/actionserver/armada_manipulation_server.cpp)

target_link_libraries(
  armada_manipulation_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  armada_manipulation_server
  ${armada_flexbe_utilities_EXPORTED_TARGETS}
)
