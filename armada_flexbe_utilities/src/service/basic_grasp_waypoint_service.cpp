#include "ros/ros.h"
#include "armada_flexbe_utilities/BasicGraspWaypoints.h"
#include "armada_flexbe_utilities/GraspPoses.h"
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>
#include <tf/transform_listener.h>

using namespace std;

class BasicGraspWaypointService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer graspWaypointService;
  double gripper_offset;
  double approach_dist;
  double retreat_dist;
  double grasp_rot_x;
  double grasp_rot_y;
  double grasp_rot_z;
  double grasp_rot_w;

  string global_frame;
  string robot_frame;

public:

  /**
   * Class Constructor.
   *
   * Constructor for BasicGraspWaypointService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  BasicGraspWaypointService(ros::NodeHandle nh) :
    nh_(nh)
  {
    graspWaypointService = nh.advertiseService("calculate_grasp_waypoints", &BasicGraspWaypointService::calculateGraspWaypoints, this);
  }

  /**
   * Generate a set of grasp waypoints (pre, target and post poses).
   *
   * Given a target Point, generate a set of waypoint poses (pre-approach, target pose, post-retreat).
   * This code uses a modification of the method for using GPD implemented in:
   * https://gist.github.com/tkelestemur/60401be131344dae98671b95d46060f8
   *
   * @param[in] req geometry_msgs/Point Position of grasp target.
   * @param[out] res armada_flexbe_utilities/GraspPoses Set of pose waypoints for grasp target.
   * @return Bool Service completion result.
   */
  bool calculateGraspWaypoints(armada_flexbe_utilities::BasicGraspWaypoints::Request &req,
                               armada_flexbe_utilities::BasicGraspWaypoints::Response &res)
  {
    nh_.getParam("/end_effector/gripper_offset", gripper_offset);
    nh_.getParam("/end_effector/approach_dist", approach_dist);
    nh_.getParam("/end_effector/retreat_dist", retreat_dist);
    nh_.getParam("/end_effector/grasping/grasp_rot_x", grasp_rot_x);
    nh_.getParam("/end_effector/grasping/grasp_rot_y", grasp_rot_y);
    nh_.getParam("/end_effector/grasping/grasp_rot_z", grasp_rot_z);
    nh_.getParam("/end_effector/grasping/grasp_rot_w", grasp_rot_w);
    nh_.getParam("/reference_frame/global_frame", global_frame);
    nh_.getParam("/reference_frame/robot_frame", robot_frame);

    armada_flexbe_utilities::GraspPoses grasp_poses;

    tf::Matrix3x3 rot_matrix_grasp_base(1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0);

    tf::Vector3 tr_grasp_base(req.position.x, req.position.y, req.position.z);
    tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);
    tf::StampedTransform tf_base_odom;

    try {
      tf::TransformListener listener;
      listener.waitForTransform(global_frame, robot_frame, ros::Time::now(), ros::Duration(3.0) );
      listener.lookupTransform(global_frame, robot_frame, ros::Time::now(), tf_base_odom);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    tf::Transform tf_grasp_odom_(tf::Quaternion(grasp_rot_x, grasp_rot_y, grasp_rot_z, grasp_rot_w), tf::Vector3(0, 0, -gripper_offset));
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
    tf::poseTFToMsg(tf_grasp_odom, grasp_poses.target);

    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -approach_dist));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, grasp_poses.pre);

    tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -retreat_dist));
    tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
    tf::poseTFToMsg(tf_aftergrasp_odom, grasp_poses.post);

    res.grasp_poses = grasp_poses;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_grasp_waypoints_service");
  ros::NodeHandle nh;

  BasicGraspWaypointService graspWaypointService = BasicGraspWaypointService(nh);
  ros::spin();

  return 0;
}
