#include <ros/ros.h>
#include "armada_flexbe_utilities/ConcatenatePointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


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
  PointCloud<PointXYZRGB> cloud_array[cloud_list_size];
  PointCloud<PointXYZRGB> temp_cloud;
  PointCloud<PointXYZRGB>::Ptr concatenated_cloud(new PointCloud<PointXYZRGB>);

  for (unsigned int i = 0; i < cloud_list_size; ++i) {
    fromROSMsg(req.cloud_list_in[i], temp_cloud);
    cloud_array[i] = temp_cloud;
  }

  *concatenated_cloud = cloud_array[0];

  if (cloud_list_size > 1) {
    for (unsigned int i = 1; i < cloud_list_size; i++) {
      *concatenated_cloud+= cloud_array[i];
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
