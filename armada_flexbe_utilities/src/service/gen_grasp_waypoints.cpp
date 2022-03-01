#include "ros/ros.h"
#include "armada_flexbe_utilities/GenGraspWaypoints.h"

bool executeCB(armada_flexbe_utilities::GenGraspWaypoints::Request  &req,
         armada_flexbe_utilities::GenGraspWaypoints::Response &res)
{
  ROS_INFO("test");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen_grasp_waypoints_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("gen_grasp_waypoints", executeCB);
  ROS_INFO("Ready to generate grasping waypoints.");
  ros::spin();

  return 0;
}
