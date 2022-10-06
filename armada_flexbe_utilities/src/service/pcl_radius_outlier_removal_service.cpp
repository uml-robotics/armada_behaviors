#include <ros/ros.h>
#include "armada_flexbe_utilities/RadiusOutlierRemoval.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace pcl;

/**
 * Filter a PointCloud2 message by performing a radius outlier removal process.
 *
 * Given a PointCloud2 message, perform a radius outlier removal process and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool radiusOutlierRemoval(armada_flexbe_utilities::RadiusOutlierRemoval::Request &req,
                          armada_flexbe_utilities::RadiusOutlierRemoval::Response &res)
{
  PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *input_cloud);

  RadiusOutlierRemoval<PointXYZRGB> outrem;
  // build the filter
  outrem.setInputCloud(input_cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);
  outrem.setKeepOrganized(true);
  // apply filter
  outrem.filter (*filtered_cloud);

  toROSMsg(*filtered_cloud, res.cloud_out);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radius_outlier_removal_service");
  ros::NodeHandle nh;

  ros::ServiceServer voxelGridFilterService = nh.advertiseService("radius_outlier_removal", radiusOutlierRemoval);
  ros::spin();

  return 0;
}
