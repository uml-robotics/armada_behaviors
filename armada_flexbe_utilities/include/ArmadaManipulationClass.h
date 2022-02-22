// ********************************************************************************************
// Author: Brian Flynn;
// Robotics Engineer - NERVE Center @ UMASS Lowell
// ArmadaManipulationClass.h
// ********************************************************************************************

#ifndef ARMADAMANIPULATIONCLASS_H
#define ARMADAMANIPULATIONCLASS_H

#include <boost/filesystem.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>

using namespace std;

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose grasp;
  geometry_msgs::Pose post;
};

class ArmadaManipulationClass
{
public:

  //Constructor
  ArmadaManipulationClass(ros::NodeHandle nh, std::string planning_group);

  //Pointers
  MoveGroupPtr move_group_ptr;
  PlanningScenePtr planning_scene_ptr;

  //Path Planning & Execution
  double cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan);
  void cartesianPick(std::vector<GraspPose> graspPose_list);
  void moveNamed(string poseName);

  //Scene Collision
  void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

};

#endif //ARMADAMANIPULATIONCLASS_H
