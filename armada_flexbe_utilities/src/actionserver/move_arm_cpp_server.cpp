#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <armada_flexbe_utilities/CartesianMoveAction.h>
#include <armada_flexbe_utilities/NamedPoseMoveAction.h>
#include "armada_flexbe_utilities/SpawnTableCollision.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class CartesianPlanningCPPAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<armada_flexbe_utilities::CartesianMoveAction> CartesianMoveServer_;
  actionlib::SimpleActionServer<armada_flexbe_utilities::NamedPoseMoveAction> MoveToNamedPoseServer_;
  armada_flexbe_utilities::CartesianMoveFeedback cartesian_move_feedback_;
  armada_flexbe_utilities::CartesianMoveResult cartesian_move_result_;
  armada_flexbe_utilities::NamedPoseMoveFeedback named_pose_move_feedback_;
  armada_flexbe_utilities::NamedPoseMoveResult named_pose_move_result_;
  ros::ServiceServer spawnTableCollisionService;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  PlanningScenePtr PlanningScenePtr_;
  double jump_threshold_;
  double eef_step_;

public:

  /**
   * Class Constructor.
   *
   * Constructor for CartesianPlanningCPPAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  CartesianPlanningCPPAction(ros::NodeHandle nh) :
    nh_(nh),
    CartesianMoveServer_(nh, "execute_cartesian_plan", boost::bind(&CartesianPlanningCPPAction::executeCartesianPlan, this, _1), false),
    MoveToNamedPoseServer_(nh, "move_to_named_pose", boost::bind(&CartesianPlanningCPPAction::moveToNamedPose, this, _1), false)
  {
    nh.getParam("/move_group/planning_group", planning_group_);

    CartesianMoveServer_.start();
    MoveToNamedPoseServer_.start();
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));
    PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);
    spawnTableCollisionService = nh.advertiseService("spawn_table_collision", &CartesianPlanningCPPAction::spawnTableObstacle, this);
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
    nh_.getParam("/move_group/jump_threshold", jump_threshold_);
    nh_.getParam("/move_group/eef_step", eef_step_);

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
    ROS_WARN_STREAM("This attempt success: " << cartesian_move_feedback_.plan_success << "%");

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

  /**
   * Spawn collision object as 'floor' to avoid table obstacle
   *
   * Add a large platform/box as a collision object to represent the robot's work surface for robot safety
   *
   * @param[in] req empty std_msgs/Empty empty data for initiating service
   * @return Bool Service completion result.
   */
  bool spawnTableObstacle(armada_flexbe_utilities::SpawnTableCollision::Request &req,
                          armada_flexbe_utilities::SpawnTableCollision::Response &res)
  {

    XmlRpc::XmlRpcValue collision_object_list;
    if( !nh_.getParam("/collision_list", collision_object_list) )
        ROS_ERROR("Still failed...");
    ROS_ASSERT(collision_object_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    int collision_objects_list_size = collision_object_list.size();

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(collision_objects_list_size);

    for (unsigned long i = 0; i < collision_objects_list_size; i++) {

      XmlRpc::XmlRpcValue collision_object = collision_object_list[i];

      std::string id = collision_object["id"];
      XmlRpc::XmlRpcValue size = collision_object["size"];
      XmlRpc::XmlRpcValue pose = collision_object["pose"];

      collision_objects[i].id = id;
      collision_objects[i].header.frame_id = MoveGroupPtr_->getPlanningFrame();

      collision_objects[i].primitives.resize(1);
      collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
      collision_objects[i].primitives[0].dimensions.resize(3);
      collision_objects[i].primitives[0].dimensions[0] = size[0];
      collision_objects[i].primitives[0].dimensions[1] = size[1];
      collision_objects[i].primitives[0].dimensions[2] = size[2];

      collision_objects[i].primitive_poses.resize(1);
      collision_objects[i].primitive_poses[0].position.x = pose[0];
      collision_objects[i].primitive_poses[0].position.y = pose[1];
      collision_objects[i].primitive_poses[0].position.z = pose[2];
      collision_objects[i].primitive_poses[0].orientation.w = 1.0;

      collision_objects[i].operation = collision_objects[i].ADD;
    }

    PlanningScenePtr_->addCollisionObjects(collision_objects);

    res.success = 1;
    return true;
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

  while(ros::ok()){
    // spin
  }

  return 0;
}
