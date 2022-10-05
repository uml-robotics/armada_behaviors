#include <ros/ros.h>
#include "armada_flexbe_utilities/PointCloudPassthroughFilter.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;

/**
 * Apply a passthrough (x,y,z) filter to a PointCloud2 message.
 *
 * Given a PointCloud2 message, apply a passthrough (x,y,z) filter and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool passthroughFilter(armada_flexbe_utilities::PointCloudPassthroughFilter::Request &req,
                       armada_flexbe_utilities::PointCloudPassthroughFilter::Response &res)
{
  ROS_WARN("Executing PassthroughFilter Service");
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *temp_cloud);

  PassThrough<PointXYZRGB> pass_x;
  pass_x.setInputCloud (temp_cloud);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (req.x_min, req.x_max);
  //pass_x.setFilterLimitsNegative(false);
  pass_x.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (req.y_min, req.y_max);
  //pass_y.setFilterLimitsNegative(false);
  pass_y.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (req.z_min, req.z_max);
  //pass_z.setFilterLimitsNegative(false);
  pass_z.filter(*temp_cloud);

  toROSMsg(*temp_cloud, res.cloud_out);
  ROS_WARN("Finishing PassthroughFilter Service");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "passthrough_filter_service");
  ros::NodeHandle nh;

  ros::ServiceServer passthroughFilterService = nh.advertiseService("passthrough_filter", passthroughFilter);
  ROS_WARN("passthrough_filter_service Ready.");
  ros::spin();

  return 0;
}
