#include <ros/ros.h>
#include "armada_flexbe_utilities/SacSegmentation.h"
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

class SacSegmentationService
{
protected:

  ros::ServiceServer sacSegmentationService;

public:

  /**
   * Class Constructor.
   *
   * Constructor for SacSegmentationService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  SacSegmentationService(ros::NodeHandle nh)
  {
    sacSegmentationService = nh.advertiseService("sac_segmentation", &SacSegmentationService::sacSegmentation, this);
  }

  /**
   * Class Destructor.
   *
   * Destructor for SacSegmentationService class.
   */
  ~SacSegmentationService(void)
  {
  }

  /**
   * Perform SAC (planar) segmentation on a PointCloud2 message.
   *
   * Given a PointCloud2 message, perform SAC (planar) segmentation and provide the resulting PointCloud2 message.
   * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool sacSegmentation(armada_flexbe_utilities::SacSegmentation::Request &req,
                       armada_flexbe_utilities::SacSegmentation::Response &res)
  {
    ROS_WARN("Executing SacSegmentation Service");
    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    *temp_cloud = transformCloud(req.cloud_in, "world");

    ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
    PointIndices::Ptr inliers_plane (new PointIndices);
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (temp_cloud);
    seg.segment (*inliers_plane, *coefficients_plane);

    ExtractIndices<PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(temp_cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*temp_cloud);

    toROSMsg(*temp_cloud, res.cloud_out);
    ROS_WARN("Finishing SacSegmentation Service");
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
  ros::init(argc, argv, "sac_segmentation_service");
  ros::NodeHandle nh;

  SacSegmentationService sacSegmentationService = SacSegmentationService(nh);
  ros::spin();

  return 0;
}
