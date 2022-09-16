#include <ros/ros.h>
#include "armada_flexbe_utilities/ConditionalOutlierRemoval.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>

using namespace pcl;

/**
 * Filter a PointCloud2 message by performing a conditional outlier removal process.
 *
 * Given a PointCloud2 message, perform a conditional outlier removal process and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool conditionalOutlierRemoval(armada_flexbe_utilities::ConditionalOutlierRemoval::Request &req,
                               armada_flexbe_utilities::ConditionalOutlierRemoval::Response &res)
{
  //ROS_WARN_STREAM("Number of points in cloud before filter: " << req.cloud_in.data.size());
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *temp_cloud);

  // perform task here

  toROSMsg(*temp_cloud, res.cloud_out);
  //ROS_WARN_STREAM("Number of points in cloud after filter: " << res.cloud_out.data.size());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conditional_outlier_removal_service");
  ros::NodeHandle nh;

  ros::ServiceServer voxelGridFilterService = nh.advertiseService("conditional_outlier_removal", conditionalOutlierRemoval);
  ros::spin();

  return 0;
}
