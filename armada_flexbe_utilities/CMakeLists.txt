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
  genmsg
  actionlib_msgs
  std_msgs
  geometry_msgs
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
  GetPointCloud.srv
  ConcatenatePointCloud.srv
  #PointCloudPassthroughFilter.srv
  #SacSegmentation.srv
  CalculateGraspWaypoints.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  CartesianMove.action
  NamedPoseMove.action
)

## Generate added msg and srv with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  gpd_ros
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_arm_cpp_server src/actionserver/move_arm_cpp_server.cpp)
add_executable(grasp_waypoint_service src/service/grasp_waypoint_service.cpp)
add_executable(concatenate_pointcloud_service src/service/concatenate_pointcloud_service.cpp)
add_executable(get_pointcloud_service src/service/get_pointcloud_service.cpp)
#add_executable(pointcloud_passthrough_filter_service src/service/pointcloud_passthrough_filter_service.cpp)
#add_executable(sac_segmentation_service src/service/sac_segmentation_service.cpp)

target_link_libraries(move_arm_cpp_server ${catkin_LIBRARIES})
target_link_libraries(grasp_waypoint_service ${catkin_LIBRARIES})
target_link_libraries(concatenate_pointcloud_service ${catkin_LIBRARIES})
target_link_libraries(get_pointcloud_service ${catkin_LIBRARIES})
#target_link_libraries(pointcloud_passthrough_filter_service ${catkin_LIBRARIES})
#target_link_libraries(sac_segmentation_service ${catkin_LIBRARIES})

add_dependencies(concatenate_pointcloud_service armada_flexbe_utilities_gencpp)
add_dependencies(get_pointcloud_service armada_flexbe_utilities_gencpp)
