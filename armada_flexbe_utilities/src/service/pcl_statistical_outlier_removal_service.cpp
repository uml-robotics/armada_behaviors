#include <ros/ros.h>
#include "armada_flexbe_utilities/StatisticalOutlierRemoval.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace pcl;

/**
 * Filter a PointCloud2 message by performing a statistical outlier removal process.
 *
 * Given a PointCloud2 message, perform a statistical outlier removal process and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 * This filter: https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool statisticalOutlierRemoval(armada_flexbe_utilities::StatisticalOutlierRemoval::Request &req,
                               armada_flexbe_utilities::StatisticalOutlierRemoval::Response &res)
{
  PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *input_cloud);

  StatisticalOutlierRemoval<PointXYZRGB> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*filtered_cloud);

  toROSMsg(*filtered_cloud, res.cloud_out);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "statistical_outlier_removal_service");
  ros::NodeHandle nh;

  ros::ServiceServer statisticalOutlierRemovalService = nh.advertiseService("statistical_outlier_removal", statisticalOutlierRemoval);
  ros::spin();

  return 0;
}
