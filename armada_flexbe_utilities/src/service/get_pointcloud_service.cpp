#include <ros/ros.h>
#include "armada_flexbe_utilities/GetPointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

using namespace pcl;

/**
 * Add current camera pointcloud msg to array of pointcloud messages.
 *
 * Given a topic and existing array of PointCloud2 messages, add another PointCloud2 message from the given topic to the array of messages.
 *
 * @param[in] req string String containing desired camera message topic.
 * @param[out] res sensor_msgs/PointCloud2 PointCloud2 message.
 * @return Bool Service completion result.
 */
bool getPointCloud(armada_flexbe_utilities::GetPointCloud::Request &req,
                   armada_flexbe_utilities::GetPointCloud::Response &res)
{
  ros::Duration timeout(5);
  sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.camera_topic, timeout);

  PointCloud<PointXYZRGB> temp_transform_cloud;
  fromROSMsg(*pointcloud2_msg, temp_transform_cloud);

  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;
  tf::TransformListener listener;

  pcl_conversions::toPCL(stamp, temp_transform_cloud.header.stamp);

  try
  {
    listener.waitForTransform("base_link", temp_transform_cloud.header.frame_id, stamp, ros::Duration(5.0));
    listener.lookupTransform("base_link", temp_transform_cloud.header.frame_id, stamp, transform);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }

  pcl_ros::transformPointCloud("base_link", temp_transform_cloud, temp_transform_cloud, listener);

  sensor_msgs::PointCloud2 transformed_pointcloud_msg;
  toROSMsg(temp_transform_cloud, transformed_pointcloud_msg);

  res.cloud_out = transformed_pointcloud_msg;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pointcloud_service");
  ros::NodeHandle nh;

  ros::ServiceServer getPointCloudService = nh.advertiseService("get_pointcloud", getPointCloud);
  ros::spin();

  return 0;
}
