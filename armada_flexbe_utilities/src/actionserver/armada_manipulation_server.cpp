#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <armada_flexbe_utilities/CartesianMoveAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;

class ArmadaManipulationAction
{
protected:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<armada_flexbe_utilities::CartesianMoveAction> CartesianMoveServer_;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  armada_flexbe_utilities::CartesianMoveFeedback feedback_;
  armada_flexbe_utilities::CartesianMoveResult result_;
  double jump_threshold_ = 0.5;            // 0.5 default, one source uses 5.0 with good results, others use 0
  double eef_step_ = 0.01;                  // 0.01 default (1 cm)

public:

  ArmadaManipulationAction(ros::NodeHandle nh, std::string planning_group) :
    CartesianMoveServer_(nh, "execute_cartesian_plan", boost::bind(&ArmadaManipulationAction::executeCartesianPlan, this, _1), false),
    planning_group_(planning_group)
  {
    CartesianMoveServer_.start();
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group));
  }

  ~ArmadaManipulationAction(void)
  {
  }

  /**
   * Plan a cartesian path.
   *
   * Plan a cartesian path along one or more set points using the MoveIt interface.
   *
   * @param[in] pose_list Container whose values make up points along the intended path.
   * @param[out] my_plan MoveIt path plan which was solved for in this function.
   * @return Percent (value from 0->100) of points along path between given points successfully planned.
   */
  double cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
  {
    moveit_msgs::RobotTrajectory trajectory;
    double success = MoveGroupPtr_->computeCartesianPath(pose_list, eef_step_, jump_threshold_, trajectory);
    success *= 100;
    my_plan.trajectory_ = trajectory;
    return success;
  }

  void executeCartesianPlan(const armada_flexbe_utilities::CartesianMoveGoalConstPtr &goal)
  {
    std::vector<geometry_msgs::Pose> pre_and_grasp_pose_list;
    std::vector<geometry_msgs::Pose> retreat_pose_list;
    moveit::planning_interface::MoveGroupInterface::Plan pre_and_grasp_pose_plan;
    moveit::planning_interface::MoveGroupInterface::Plan retreat_pose_plan;

    armada_flexbe_utilities::GraspPosesList poses_list;
    poses_list = goal->pose_list;

    bool temp_flag;

    unsigned long n = sizeof(poses_list);
    for (unsigned long i = 0; i < n; ++i) {
      pre_and_grasp_pose_list.clear();
      //pre_and_grasp_pose_list.push_back(poses_list.poses[i].pre);
      //pre_and_grasp_pose_list.push_back(poses_list.poses[i].target);
      retreat_pose_list.clear();
      //retreat_pose_list.push_back(poses_list[i].post);
      // get cartesian plan for pre and actual grasp, if good execute and close gripper
      double pre_and_grasp_success = cartesianPlan(pre_and_grasp_pose_list, pre_and_grasp_pose_plan);
      if (pre_and_grasp_success == 100) {
        MoveGroupPtr_->execute(pre_and_grasp_pose_plan);
        ros::Duration(0.5).sleep();
        //setGripper(0.5);
        temp_flag = 1;
      }
      ros::Duration(.5).sleep();
      // get cartesian plan for post grasp, if good execute
      if (temp_flag) {
        double post_success = cartesianPlan(retreat_pose_list, retreat_pose_plan);
        if (post_success == 100) {
          //move_group_ptr->execute(retreat_pose_plan);
          ros::Duration(0.5).sleep();
          break;
        }
      }
      ros::Duration(0.5).sleep();
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armada_manipulation_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while(ros::ok())
  {
    // spin until shutdown
  }

  return 0;
}
