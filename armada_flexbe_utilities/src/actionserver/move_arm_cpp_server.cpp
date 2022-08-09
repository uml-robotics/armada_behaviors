#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <armada_flexbe_utilities/CartesianMoveAction.h>
#include <armada_flexbe_utilities/NamedPoseMoveAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;

class CartesianPlanningCPPAction
{
protected:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<armada_flexbe_utilities::CartesianMoveAction> CartesianMoveServer_;
  actionlib::SimpleActionServer<armada_flexbe_utilities::NamedPoseMoveAction> MoveToNamedPoseServer_;
  armada_flexbe_utilities::CartesianMoveFeedback cartesian_move_feedback_;
  armada_flexbe_utilities::CartesianMoveResult cartesian_move_result_;
  armada_flexbe_utilities::NamedPoseMoveFeedback named_pose_move_feedback_;
  armada_flexbe_utilities::NamedPoseMoveResult named_pose_move_result_;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  double jump_threshold_;
  double eef_step_;

public:

  /**
   * Class Constructor.
   *
   * Constructor for CartesianPlanningCPPAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   * @param[in] planning_group MoveIt manipulator planning group.
   */
  CartesianPlanningCPPAction(ros::NodeHandle nh) :
    CartesianMoveServer_(nh, "execute_cartesian_plan", boost::bind(&CartesianPlanningCPPAction::executeCartesianPlan, this, _1), false),
    MoveToNamedPoseServer_(nh, "move_to_named_pose", boost::bind(&CartesianPlanningCPPAction::moveToNamedPose, this, _1), false)
  {
    nh.getParam("/move_group/planning_group", planning_group_);

    CartesianMoveServer_.start();
    MoveToNamedPoseServer_.start();
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));
  }

  /**
   * Class Destructor.
   *
   * Destructor for CartesianPlanningCPPAction class.
   */
  ~CartesianPlanningCPPAction(void)
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
    nh.getParam("/move_group/jump_threshold", jump_threshold_);
    nh.getParam("/move_group/eef_step", eef_step_);

    moveit_msgs::RobotTrajectory trajectory;
    double success = MoveGroupPtr_->computeCartesianPath(pose_list, eef_step_, jump_threshold_, trajectory);
    success *= 100;
    my_plan.trajectory_ = trajectory;
    ros::Duration(0.5).sleep();   // this was in here previously, testing to see if necessary
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

    cartesian_move_feedback_.plan_success = cartesianPlan(waypoints, plan);
    CartesianMoveServer_.publishFeedback(cartesian_move_feedback_);
    ROS_WARN_STREAM("Pre/Grasp Success: " << cartesian_move_feedback_.plan_success << "%");

    if (cartesian_move_feedback_.plan_success >= 100) {
      MoveGroupPtr_->execute(plan);
      ros::Duration(0.5).sleep();
      cartesian_move_result_.execution_success = 1;
    } else {
      cartesian_move_result_.execution_success = 0;
    }
    CartesianMoveServer_.setSucceeded(cartesian_move_result_);
  }

  /**
   * Move to one or more pre-defined, named positions.
   *
   * Plan and move to one or more pre-defined, named positions using the MoveIt interface.
   *
   * @param[in] pose_names string[] named positions as defined in robot's SRDF.
   */
  void moveToNamedPose(const armada_flexbe_utilities::NamedPoseMoveGoalConstPtr &goal)
  {
    std::vector<std::string> poses;
    poses.insert(poses.begin(), std::begin(goal->pose_names), std::end(goal->pose_names));

    unsigned long n = poses.size();
    for (unsigned long i = 0; i < n; i++) {
      MoveGroupPtr_->setNamedTarget(poses[i]);
      MoveGroupPtr_->move();
    }
    named_pose_move_result_.execution_success = 1;
    MoveToNamedPoseServer_.setSucceeded(named_pose_move_result_);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_planning_cpp_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  CartesianPlanningCPPAction cartesian_planning_cpp_server(nh);
  ROS_WARN("cartesian_planning_cpp_server Ready.");
  ros::spin();

  return 0;
}
