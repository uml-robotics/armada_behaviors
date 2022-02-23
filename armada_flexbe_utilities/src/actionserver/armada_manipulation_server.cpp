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
    // do things
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
