#include <ros/ros.h>
#include "armada_flexbe_utilities/VoxelGridFilter.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;

/**
 * Downsample a PointCloud2 message by applying a voxelgrid filter.
 *
 * Given a PointCloud2 message, apply a voxelgrid filter and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool voxelGridFilter(armada_flexbe_utilities::VoxelGridFilter::Request &req,
                     armada_flexbe_utilities::VoxelGridFilter::Response &res)
{
  ROS_WARN_STREAM("Number of points in cloud before filter: " << req.cloud_in.data.size());
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *temp_cloud);

  // Create the filtering object
    VoxelGrid<PointXYZRGB> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*filtered_cloud);

  toROSMsg(*filtered_cloud, res.cloud_out);
  ROS_WARN_STREAM("Number of points in cloud after filter: " << res.cloud_out.data.size());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voxel_grid_filter_service");
  ros::NodeHandle nh;

  ros::ServiceServer voxelGridFilterService = nh.advertiseService("voxelgrid_filter", voxelGridFilter);
  ros::spin();

  return 0;
}
