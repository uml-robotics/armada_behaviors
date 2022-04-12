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
  geometry_msgs
  genmsg
  actionlib_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  gpd_ros
  tf
  pcl_conversions
  pcl_ros
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GraspPoses.msg
  GraspPosesList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GenGraspWaypoints.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  CartesianMove.action
)

add_action_files(
  DIRECTORY action
  FILES
  NamedPoseMove.action
)

add_action_files(
  DIRECTORY action
  FILES
  PointcloudProcessing.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  gpd_ros
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

add_executable(cartesian_planning_cpp_server src/actionserver/cartesian_planning_cpp_server.cpp)
add_executable(pointcloud_processing_cpp_server src/actionserver/pointcloud_processing_cpp_server.cpp)
add_executable(gen_grasp_waypoints src/service/gen_grasp_waypoints.cpp)

target_link_libraries(cartesian_planning_cpp_server ${catkin_LIBRARIES})
target_link_libraries(pointcloud_processing_cpp_server ${catkin_LIBRARIES})
target_link_libraries(gen_grasp_waypoints ${catkin_LIBRARIES})

#add_dependencies(armada_manipulation_server ${armada_flexbe_utilities_EXPORTED_TARGETS})
#add_dependencies(gen_grasp_waypoints ${armada_flexbe_utilities_EXPORTED_TARGETS})
