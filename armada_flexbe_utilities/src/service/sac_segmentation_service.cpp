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
  fromROSMsg(req.cloud_in, *temp_cloud);

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sac_segmentation_service");
  ros::NodeHandle nh;

  ros::ServiceServer sacSegmentationService = nh.advertiseService("sac_segmentation", sacSegmentation);
  ROS_WARN("sac_segmentation_service Ready.");
  ros::spin();

  return 0;
}
