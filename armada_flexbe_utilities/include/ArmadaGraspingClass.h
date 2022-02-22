// ********************************************************************************************
// Author: Brian Flynn;
// Robotics Engineer - NERVE Center @ UMASS Lowell
// ArmadaManipulationClass.h
// ********************************************************************************************

#ifndef ARMADAMANIPULATIONCLASS_H
#define ARMADAMANIPULATIONCLASS_H

#include <boost/filesystem.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>

using namespace std;


typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose grasp;
  geometry_msgs::Pose post;
};

class ArmadaGraspingClass
{
public:

  //Constructor
  ArmadaGraspingClass(ros::NodeHandle nh);

  //Pointers
  TransformListenerPtr transform_listener_ptr;

  //Grasp Goal Generation
  std::vector<GraspPose> createPickingEEFPoseList(gpd_ros::GraspConfigList candidates);
  GraspPose createPickingEEFPose(gpd_ros::GraspConfig grasp_msg);

};

#endif //ARMADAMANIPULATIONCLASS_H
