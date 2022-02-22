#include "ArmadaManipulationClass.h"

// plan a cartesian path
double ArmadaManipulationClass::cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
  moveit_msgs::RobotTrajectory trajectory;
  double success = move_group_ptr->computeCartesianPath(pose_list, eef_step, jump_threshold, trajectory);
  success *= 100;
  my_plan.trajectory_ = trajectory;
  ros::Duration(0.5).sleep();
  return success;
}

// Using helper function createPickingEEFPose graspPoses, plan and execute a picking maneuver
void ArmadaManipulationClass::cartesianPick(std::vector<GraspPose> graspPose_list)
{
  std::vector<geometry_msgs::Pose> pre_and_grasp_pose_list;
  std::vector<geometry_msgs::Pose> retreat_pose_list;
  moveit::planning_interface::MoveGroupInterface::Plan pre_and_grasp_pose_plan;
  moveit::planning_interface::MoveGroupInterface::Plan retreat_pose_plan;

  bool approach_complete;

  unsigned long n = graspPose_list.size();
  for (unsigned long i = 0; i < n; ++i) {
    pre_and_grasp_pose_list.clear();
    pre_and_grasp_pose_list.push_back(graspPose_list[i].pre);
    pre_and_grasp_pose_list.push_back(graspPose_list[i].grasp);
    retreat_pose_list.clear();
    retreat_pose_list.push_back(graspPose_list[i].post);
    // get cartesian plan for pre and actual grasp, if good execute and close gripper
    double pre_and_grasp_success = cartesianPlan(pre_and_grasp_pose_list, pre_and_grasp_pose_plan);
    if (pre_and_grasp_success == 100) {
      move_group_ptr->execute(pre_and_grasp_pose_plan);
      ros::Duration(0.5).sleep();
      setGripper(0.5);
      approach_complete = 1;
    }
    ros::Duration(.5).sleep();
    // get cartesian plan for post grasp, if good execute
    if (approach_complete) {
      double post_success = cartesianPlan(retreat_pose_list, retreat_pose_plan);
      if (post_success == 100) {
        move_group_ptr->execute(retreat_pose_plan);
        ros::Duration(0.5).sleep();
        break;
      }
    }
    ros::Duration(0.5).sleep();
  }
}

void ArmadaManipulationClass::moveNamed(string poseName)
{
  move_group_ptr->setNamedTarget(poseName);
  move_group_ptr->move();
  ros::Duration(1.0).sleep();
}
