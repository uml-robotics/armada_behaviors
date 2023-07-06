#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "armada_flexbe_utilities/SpawnTableCollision.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class SpawnTableCollisionService
{
protected:

  struct object_dimensions {
    double x_len;
    double y_len;
    double z_len;
    double x_pos;
    double y_pos;
    double z_pos;
  };

  ros::NodeHandle nh_;
  ros::ServiceServer spawnTableCollisionService;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  PlanningScenePtr PlanningScenePtr_;
  object_dimensions table;

public:

  /**
   * Class Constructor.;
   *
   * Constructor for SpawnTableCollisionService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  SpawnTableCollisionService(ros::NodeHandle nh) :
    nh_(nh)
  {
    nh.getParam("/move_group/planning_group", planning_group_);
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));

    PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);
    spawnTableCollisionService = nh.advertiseService("spawn_table_collision", &SpawnTableCollisionService::spawnTableObstacle, this);
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
    nh_.getParam("/collision/table/dimension/x", table.x_len);
    nh_.getParam("/collision/table/dimension/y", table.y_len);
    nh_.getParam("/collision/table/dimension/z", table.z_len);
    nh_.getParam("/collision/table/pose/x", table.x_pos);
    nh_.getParam("/collision/table/pose/y", table.y_pos);
    nh_.getParam("/collision/table/pose/z", table.z_pos);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = "box1";
    collision_objects[0].header.frame_id = MoveGroupPtr_->getPlanningFrame();

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = table.x_len;
    collision_objects[0].primitives[0].dimensions[1] = table.y_len;
    collision_objects[0].primitives[0].dimensions[2] = table.z_len;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = table.x_pos;
    collision_objects[0].primitive_poses[0].position.y = table.y_pos;
    collision_objects[0].primitive_poses[0].position.z = table.z_pos;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    PlanningScenePtr_->addCollisionObjects(collision_objects);

    res.success = 1;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_table_collision_service");
  ros::NodeHandle nh;

  SpawnTableCollisionService spawnTableCollisionService = SpawnTableCollisionService(nh);
  ros::spin();

  return 0;
}
