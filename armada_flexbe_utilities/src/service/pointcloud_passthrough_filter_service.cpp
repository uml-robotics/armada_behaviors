#include <ros/ros.h>
#include "armada_flexbe_utilities/PointCloudPassthroughFilter.h"
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

class PassthroughFilterService
{
protected:

  ros::ServiceServer passthroughFilterService;

public:

  /**
   * Class Constructor.
   *
   * Constructor for PassthroughFilterService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  PassthroughFilterService(ros::NodeHandle nh)
  {
    passthroughFilterService = nh.advertiseService("passthrough_filter", &PassthroughFilterService::passthroughFilter, this);
  }

  /**
   * Class Destructor.
   *
   * Destructor for PassthroughFilterService class.
   */
  ~PassthroughFilterService(void)
  {
  }

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
    *temp_cloud = transformCloud(req.cloud_in, "world");

    // Passthrough filter to limit to work area
    PassThrough<PointXYZRGB> pass_x;
    pass_x.setInputCloud (temp_cloud);
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (req.x_min, req.x_max);
    pass_x.filter(*temp_cloud);

    PassThrough<PointXYZRGB> pass_y;
    pass_y.setInputCloud (temp_cloud);
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (req.y_min, req.y_max);
    pass_y.filter(*temp_cloud);

    PassThrough<PointXYZRGB> pass_z;
    pass_z.setInputCloud (temp_cloud);
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (req.z_min, req.z_max);
    pass_z.filter(*temp_cloud);

    toROSMsg(*temp_cloud, res.cloud_out);
    ROS_WARN("Finishing PassthroughFilter Service");
    return true;
  }

  /**
   * Transform a PointCloud2 message into a PointCloud object.
   *
   * Given a PointCloud2 message and a target frame, transform it into a PointCloud<T> object in the target frame.
   *
   * @param[in] cloud sensor_msgs::PointCloud2 A PointCloud2 message.
   * @param[in] target_frame std::string The target object transformation frame.
   * @return PointCloud<PointXYZRGB> A PointCloud object.
   */
    PointCloud<PointXYZRGB> transformCloud(sensor_msgs::PointCloud2 cloud, std::string target_frame){
    PointCloud<PointXYZRGB> temp_cloud;
    fromROSMsg(cloud, temp_cloud);
    boost::shared_ptr<tf::TransformListener> listener;

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, temp_cloud.header.stamp);

    try
    {
      listener->waitForTransform(target_frame, temp_cloud.header.frame_id, stamp, ros::Duration(10.0));
      listener->lookupTransform(target_frame, temp_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
      ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud(target_frame, temp_cloud, temp_cloud, *listener);

    return temp_cloud;
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
