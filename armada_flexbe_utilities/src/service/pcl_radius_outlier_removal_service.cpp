#include <ros/ros.h>
#include "armada_flexbe_utilities/RadiusOutlierRemoval.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace pcl;

class RadiusOutlierRemovalService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer radiusOutlierRemovalService;
  double searchRadius;
  int minNeighborRadius;

public:

  /**
   * Class Constructor.
   *
   * Constructor for SetGripperService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  RadiusOutlierRemovalService(ros::NodeHandle nh) :
    nh_(nh)
  {
    radiusOutlierRemovalService = nh.advertiseService("radius_outlier_removal", &RadiusOutlierRemovalService::radiusOutlierRemoval, this);
  }

  /**
   * Filter a PointCloud2 message by performing a radius outlier removal process.
   *
   * Given a PointCloud2 message, perform a radius outlier removal process and provide the resulting PointCloud2 message.
   * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
   * This filter (split into two separate services): https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool radiusOutlierRemoval(armada_flexbe_utilities::RadiusOutlierRemoval::Request &req,
                            armada_flexbe_utilities::RadiusOutlierRemoval::Response &res)
  {
    nh_.getParam("/filters/radius_outlier_removal/searchRadius", searchRadius);
    nh_.getParam("/filters/radius_outlier_removal/minNeighborRadius", minNeighborRadius);

    PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *input_cloud);

    RadiusOutlierRemoval<PointXYZRGB> outrem;
    outrem.setInputCloud(input_cloud);
    outrem.setRadiusSearch(searchRadius);
    outrem.setMinNeighborsInRadius(minNeighborRadius);
    outrem.setKeepOrganized(true);
    outrem.filter (*filtered_cloud);

    toROSMsg(*filtered_cloud, res.cloud_out);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radius_outlier_removal_service");
  ros::NodeHandle nh;

  RadiusOutlierRemovalService radiusOutlierRemovalService = RadiusOutlierRemovalService(nh);
  ros::spin();

  return 0;
}
