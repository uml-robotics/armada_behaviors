#include <ros/ros.h>
#include "armada_flexbe_utilities/PCLPlaneSegmentation.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;

class PlaneSegmentationService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer planeSegmentationService;
  int maxIterations;
  double distanceThreshold;

public:

  /**
   * Class Constructor.
   *
   * Constructor for PlaneSegmentationService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  PlaneSegmentationService(ros::NodeHandle nh) :
    nh_(nh)
  {
    planeSegmentationService = nh.advertiseService("plane_segmentation", &PlaneSegmentationService::planeSegmentation, this);
  }

  /**
   * Perform SAC (planar) segmentation on a PointCloud2 message.
   *
   * Given a PointCloud2 message, perform SAC (planar) segmentation and provide the resulting PointCloud2 message.
   * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
   * This filter/segmentation: https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool planeSegmentation(armada_flexbe_utilities::PCLPlaneSegmentation::Request &req,
                         armada_flexbe_utilities::PCLPlaneSegmentation::Response &res)
  {
    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr objects_cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr plane_cloud(new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *temp_cloud);

    nh_.getParam("/filters/plane_segmentation/maxIterations", maxIterations);
    nh_.getParam("/filters/plane_segmentation/distanceThreshold", distanceThreshold);

    ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
    PointIndices::Ptr inliers_plane (new PointIndices);
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (temp_cloud);
    seg.segment (*inliers_plane, *coefficients_plane);

    ExtractIndices<PointXYZRGB> extract_object_indices;
    extract_object_indices.setInputCloud(temp_cloud);
    extract_object_indices.setIndices(inliers_plane);
    extract_object_indices.setNegative(true);
    extract_object_indices.filter(*objects_cloud);

    ExtractIndices<PointXYZRGB> extract_plane_indices;
    extract_plane_indices.setInputCloud(temp_cloud);
    extract_plane_indices.setIndices(inliers_plane);
    extract_plane_indices.setNegative(false);
    extract_plane_indices.filter(*plane_cloud);

    toROSMsg(*objects_cloud, res.objects_cloud_out);
    toROSMsg(*plane_cloud, res.plane_cloud_out);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plane_segmentation_service");
  ros::NodeHandle nh;

  PlaneSegmentationService planeSegmentationService = PlaneSegmentationService(nh);
  ros::spin();

  return 0;
}
