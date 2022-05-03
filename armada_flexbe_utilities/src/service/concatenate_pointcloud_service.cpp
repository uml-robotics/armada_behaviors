#include <ros/ros.h>
#include "armada_flexbe_utilities/ConcatenatePointCloud.h"
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
 * Concatenate an array of PointCloud2 into a single PointCloud2.
 *
 * Given an array of PointCloud2 messages, concatenate into a single PointCloud2 message for publishing.
 *
 * @param[in] req sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool concatenatePointCloud(armada_flexbe_utilities::ConcatenatePointCloud::Request &req,
                           armada_flexbe_utilities::ConcatenatePointCloud::Response &res)
{
  ROS_WARN("Executing ConcatenatePointCloud Service");
  unsigned int cloud_list_size = req.cloud_list_in.size();
  ROS_WARN("PointCloud2 List Size: %d ...", cloud_list_size);
  PointCloud<PointXYZRGB> cloud_array[cloud_list_size];
  PointCloud<PointXYZRGB>::Ptr concatenated_cloud(new PointCloud<PointXYZRGB>);

  // stuff for cloud transform
  PointCloud<PointXYZRGB> temp_transform_cloud;
  tf::TransformListener listener;
  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;

  for (unsigned int i = 0; i < cloud_list_size; ++i) {
    fromROSMsg(req.cloud_list_in[i], temp_transform_cloud);
    pcl_conversions::toPCL(stamp, temp_transform_cloud.header.stamp);

    try
    {
      listener.waitForTransform("world", temp_transform_cloud.header.frame_id, stamp, ros::Duration(10.0));
      listener.lookupTransform("world", temp_transform_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
      ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_transform_cloud, temp_transform_cloud, listener);
    cloud_array[i] = temp_transform_cloud;
  }

  *concatenated_cloud = cloud_array[0];

  if (cloud_list_size > 1) {
    for (unsigned int i = 1; i < cloud_list_size; i++) {
      *concatenated_cloud+= cloud_array[i];
      ROS_WARN("Concatenating cloud number: %d ...", i);
    }
  }
  ROS_WARN("Finishing ConcatenatePointCloud Service");
  toROSMsg(*concatenated_cloud, res.cloud_out);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "concatenate_pointcloud_service");
  ros::NodeHandle nh;

  ros::ServiceServer concatenatePointCloudService = nh.advertiseService("concatenate_pointcloud", concatenatePointCloud);
  ROS_WARN("concatenate_pointcloud_service Ready.");
  ros::spin();

  return 0;
}
