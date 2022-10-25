#include <ros/ros.h>
#include "armada_flexbe_utilities/PointCloudPassthroughFilter.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;

class PassthroughFilterService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer passthroughFilterService;
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;

public:

  /**
   * Class Constructor.
   *
   * Constructor for PassthroughFilterService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  PassthroughFilterService(ros::NodeHandle nh) :
    nh_(nh)
  {
    passthroughFilterService = nh.advertiseService("passthrough_filter", &PassthroughFilterService::passthroughFilter, this);
  }

  /**
   * Apply a passthrough (x,y,z) filter to a PointCloud2 message.
   *
   * Given a PointCloud2 message, apply a passthrough (x,y,z) filter and provide the resulting PointCloud2 message.
   * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
   * This filter: https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool passthroughFilter(armada_flexbe_utilities::PointCloudPassthroughFilter::Request &req,
                         armada_flexbe_utilities::PointCloudPassthroughFilter::Response &res)
  {
    nh_.getParam("/filters/passthrough/x", x_min);
    nh_.getParam("/filters/passthrough/x", x_max);
    nh_.getParam("/filters/passthrough/y", y_min);
    nh_.getParam("/filters/passthrough/y", y_max);
    nh_.getParam("/filters/passthrough/z", z_min);
    nh_.getParam("/filters/passthrough/z", z_max);

    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *temp_cloud);

    PassThrough<PointXYZRGB> pass_x;
    pass_x.setInputCloud (temp_cloud);
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (x_min, x_max);
    //pass_x.setFilterLimitsNegative(false);
    pass_x.filter(*temp_cloud);

    PassThrough<PointXYZRGB> pass_y;
    pass_y.setInputCloud (temp_cloud);
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (y_min, y_max);
    pass_y.filter(*temp_cloud);

    PassThrough<PointXYZRGB> pass_z;
    pass_z.setInputCloud (temp_cloud);
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (z_min, z_max);
    pass_z.filter(*temp_cloud);

    toROSMsg(*temp_cloud, res.cloud_out);
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "passthrough_filter_service");
  ros::NodeHandle nh;

  PassthroughFilterService passthroughFilterService = PassthroughFilterService(nh);
  ros::spin();

  return 0;
}
