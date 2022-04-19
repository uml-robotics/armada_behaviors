#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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
  double jump_threshold_ = 0.5;               // 0.5 default (works well for me), one source uses 5.0 with good results, others use 0
  double eef_step_ = 0.01;                    // 0.01 default (1 cm)

public:

  /**
   * Class Constructor.
   *
   * Constructor for AmadaManipulationAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   * @param[in] planning_group MoveIt manipulator planning group.
   */
  ArmadaManipulationAction(ros::NodeHandle nh, std::string planning_group) :
    CartesianMoveServer_(nh, "execute_cartesian_plan", boost::bind(&ArmadaManipulationAction::executeCartesianPlan, this, _1), false),
    planning_group_(planning_group)
  {
    CartesianMoveServer_.start();
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group));
  }

  /**
   * Class Destructor.
   *
   * Destructor for AmadaManipulationAction class.
   */
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
    my_plan.trajectory_ = trajectory;
    return success;
  }

  /**
   * Execute a cartesian path plan.
   *
   * Execute a cartesian movement planned along one or more set points using the MoveIt interface.
   *
   * @param goal CartesianMove action goal: geometry_msgs/Pose[], a set of waypoints along a path.
   * @return Success of motion plan execution.
   */
  void executeCartesianPlan(const armada_flexbe_utilities::CartesianMoveGoalConstPtr &goal)
  {
    std::vector<geometry_msgs::Pose> waypoints;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    waypoints.insert(waypoints.begin(), std::begin(goal->grasp_waypoints), std::end(goal->grasp_waypoints));

    unsigned long n = waypoints.size();
    for (unsigned long i = 0; i < n; ++i) {
      feedback_.plan_success = cartesianPlan(waypoints, plan);
      CartesianMoveServer_.publishFeedback(feedback_);
      if (feedback_.plan_success == 1.00) {
        MoveGroupPtr_->execute(plan);
        ros::Duration(0.5).sleep();
      }
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