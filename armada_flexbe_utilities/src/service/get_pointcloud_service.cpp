#include <ros/ros.h>
#include "armada_flexbe_utilities/GetPointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

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
  ROS_WARN("Executing GetPointCloud Service");
  ros::Duration timeout(10);
  sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.camera_topic, timeout);

  res.cloud_out = *pointcloud2_msg;
  ROS_WARN("Finished GetPointCloud Service");
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pointcloud_service");
  ros::NodeHandle nh;

  ros::ServiceServer getPointCloudService = nh.advertiseService("get_pointcloud", getPointCloud);
  ROS_WARN("get_pointcloud_service Ready.");
  ros::spin();

  return 0;
}
