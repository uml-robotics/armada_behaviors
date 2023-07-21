#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <geometry_msgs/Pose.h>
#include "armada_flexbe_utilities/SpawnCollision.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class SpawnCollisionService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer spawnCollisionService;
  std::string planning_group_;
  MoveGroupPtr MoveGroupPtr_;
  PlanningScenePtr PlanningScenePtr_;

public:

  /**
   * Class Constructor.;
   *
   * Constructor for SpawnCollisionService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  SpawnCollisionService(ros::NodeHandle nh) :
    nh_(nh)
  {
    nh.getParam("/move_group/planning_group", planning_group_);
    MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));

    PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);
    spawnCollisionService = nh.advertiseService("spawn_collision", &SpawnCollisionService::spawnObstacle, this);
  }

  /**
   * Spawn collision objects for robot/user safety
   *
   * Add a large platform/box as a collision object to represent the robot's work surface for robot safety
   *
   * @param[in] req empty std_msgs/Empty empty data for initiating service
   * @return Bool Service completion result.
   */
  bool spawnObstacle(armada_flexbe_utilities::SpawnCollision::Request &req,
                           armada_flexbe_utilities::SpawnCollision::Response &res)
  {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    XmlRpc::XmlRpcValue collision_list;

    nh_.getParam("collision_list", collision_list);
    int size = collision_list.size();
    collision_objects.resize(size);

    for (int i = 0; i < size; i++)
    {
      XmlRpc::XmlRpcValue sublist = collision_list[i];

      collision_objects[i].id = sublist["id"].toXml();
      collision_objects[i].header.frame_id = MoveGroupPtr_->getPlanningFrame();

      collision_objects[i].primitives.resize(1);
      collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
      collision_objects[i].primitives[0].dimensions.resize(3);
      collision_objects[i].primitive_poses.resize(1);

      collision_objects[i].primitives[0].dimensions[0] = sublist["size"][0];
      collision_objects[i].primitives[0].dimensions[1] = sublist["size"][1];
      collision_objects[i].primitives[0].dimensions[2] = sublist["size"][2];

      collision_objects[i].primitive_poses[0].position.x = sublist["position"][0];
      collision_objects[i].primitive_poses[0].position.y = sublist["position"][1];
      collision_objects[i].primitive_poses[0].position.z = sublist["position"][2];
      
      tf2::Quaternion q;
      q.setRPY(sublist["orientation"][0], sublist["orientation"][1], sublist["orientation"][2]);
      q = q.normalize();
      collision_objects[i].primitive_poses[0].orientation.x = q.x();
      collision_objects[i].primitive_poses[0].orientation.y = q.y();
      collision_objects[i].primitive_poses[0].orientation.z = q.z();
      collision_objects[i].primitive_poses[0].orientation.w = q.w();

      collision_objects[i].operation = collision_objects[i].ADD;
    }
    PlanningScenePtr_->addCollisionObjects(collision_objects);

    res.success = 1;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spawn_collision_service");
  ros::NodeHandle nh;

  SpawnCollisionService spawnCollisionService = SpawnCollisionService(nh);
  ros::spin();

  return 0;
}
