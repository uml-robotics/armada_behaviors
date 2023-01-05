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
  };

  ros::NodeHandle nh_;
  ros::ServiceServer spawnTableCollisionService;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  PlanningScenePtr PlanningScenePtr_;
  object_dimensions table;

public:

  /**
   * Class Constructor.
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
    nh_.getParam("/collision/table/x", table.x_len);
    nh_.getParam("/collision/table/y", table.y_len);
    nh_.getParam("/collision/table/z", table.z_len);

    moveit_msgs::CollisionObject collision_surface;
    collision_surface.header.frame_id = MoveGroupPtr_->getPlanningFrame();

    // The id of the object is used to identify it.
    collision_surface.id = "box1";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = table.x_len;
    primitive.dimensions[primitive.BOX_Y] = table.y_len;
    primitive.dimensions[primitive.BOX_Z] = table.z_len;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    double table_offset = table.z_len / 2;
    table_pose.position.z = 0 - table_offset;

    collision_surface.primitives.push_back(primitive);
    collision_surface.primitive_poses.push_back(table_pose);
    collision_surface.operation = collision_surface.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_surface);
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
