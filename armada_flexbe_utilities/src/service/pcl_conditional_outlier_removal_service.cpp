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
 * This filter (split into two separate services): https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool conditionalOutlierRemoval(armada_flexbe_utilities::ConditionalOutlierRemoval::Request &req,
                               armada_flexbe_utilities::ConditionalOutlierRemoval::Response &res)
{
  PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *input_cloud);

  ConditionAnd<PointXYZRGB>::Ptr range_cond (new ConditionAnd<PointXYZRGB> ());
  range_cond->addComparison (FieldComparison<PointXYZRGB>::ConstPtr (new FieldComparison<PointXYZRGB> ("z", ComparisonOps::GT, 0.0)));
  range_cond->addComparison (FieldComparison<PointXYZRGB>::ConstPtr (new FieldComparison<PointXYZRGB> ("z", ComparisonOps::LT, 0.8)));
  ConditionalRemoval<PointXYZRGB> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (input_cloud);
  condrem.setKeepOrganized(true);
  condrem.filter (*filtered_cloud);

  toROSMsg(*filtered_cloud, res.cloud_out);
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
