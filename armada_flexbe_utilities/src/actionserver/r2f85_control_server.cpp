#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>


class R2f85GripperControlAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> R2f85GripperCommandServer_;
  control_msgs::GripperCommandFeedback r2f85_gripper_feedback_;
  control_msgs::GripperCommandResult r2f85_gripper_result_;

public:

  /**
   * Class Constructor.
   *
   * Constructor for R2f85GripperControlAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  R2f85GripperControlAction(ros::NodeHandle nh) :
    nh_(nh),
    R2f85GripperCommandServer_(nh, "r2f85_gripper_command", boost::bind(&R2f85GripperControlAction::r2f85GripperCommand, this, _1), false)
  {
    R2f85GripperCommandServer_.start();
  }

  /**
   * Plan a cartesian path.
   *
   * Plan a cartesian path along one or more set points using the MoveIt interface.
   *
   * @param goal GripperCommand action goal: control_msgs/GripperCommand, a gripper command target position.
   * @return Success of motion plan execution.
   */
  void r2f85GripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal)
  {
    // do something
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "r2f85_gripper_command_action_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  R2f85GripperControlAction cartesian_planning_cpp_server(nh);
  ROS_WARN("r2f85_gripper_command_action_server Ready.");

  while(ros::ok()){
    // spin
  }

  return 0;
}
