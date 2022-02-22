#include "ArmadaGraspingClass.h"

std::vector<GraspPose> ArmadaGraspingClass::createPickingEEFPoseList(gpd_ros::GraspConfigList candidates)
{
  std::vector<GraspPose> grasp_pose_list;
  grasp_pose_list.clear();
  unsigned long n = candidates.grasps.size();
  for (unsigned long i = 0; i < n; ++i) {
    grasp_pose_list.push_back(createPickingEEFPose(candidates.grasps[i]));
  }
  return grasp_pose_list;
}

GraspPose ArmadaGraspingClass::createPickingEEFPose(gpd_ros::GraspConfig grasp_msg)
{
  GraspPose thisGrasp;

  tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

  tf::Vector3 tr_grasp_base(grasp_msg.position.x, grasp_msg.position.y, grasp_msg.position.z);
  tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);
  tf::StampedTransform tf_base_odom;

  try {
    transform_listener_ptr->waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0) );
    transform_listener_ptr->lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
  } catch (tf::TransformException err) {
    ROS_ERROR("%s", err.what());
  }

  tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, -M_PI/4 - M_PI/16, 1), tf::Vector3(0, 0, -grasp_offset));
  tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
  tf::poseTFToMsg(tf_grasp_odom, thisGrasp.grasp);

  tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -pregrasp_dist));
  tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
  tf::poseTFToMsg(tf_pregrasp_odom, thisGrasp.pre);

  tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -pregrasp_dist));
  tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
  tf::poseTFToMsg(tf_aftergrasp_odom, thisGrasp.post);

  return thisGrasp;
}
