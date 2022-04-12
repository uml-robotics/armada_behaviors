#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

void getPointCloud()
{

}

void concatenatePointCloud()
{

}

void sacSegmentation()
{

}

PointCloud<PointXYZRGB> Perception::transformCloud(sensor_msgs::PointCloud2 cloud){
  PointCloud<PointXYZRGB> temp_cloud;
  fromROSMsg(cloud, temp_cloud);

  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;

  pcl_conversions::toPCL(stamp, temp_cloud.header.stamp);

  // wait for and then apply transform
  try
  {
    transform_listener_ptr->waitForTransform("world", temp_cloud.header.frame_id, stamp, ros::Duration(10.0));
    transform_listener_ptr->lookupTransform("world", temp_cloud.header.frame_id, stamp, transform);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }

  // transform the cloud to the world frame but use the same pointcloud object
  pcl_ros::transformPointCloud("world", temp_cloud, temp_cloud, *transform_listener_ptr);

  return temp_cloud;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_processing_service");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::ServiceServer getPointCloudService = nh.advertiseService("get_pointcloud", getPointCloud);
  ROS_INFO("Ready to retrieve pointcloud.");

  ros::ServiceServer concatenatePointCloudService = nh.advertiseService("concatenate_pointcloud", concatenatePointCloud);
  ROS_INFO("Ready to retrieve pointcloud.");

  ros::ServiceServer sacSegmentationService = nh.advertiseService("sac_segmentation", sacSegmentation);
  ROS_INFO("Ready to retrieve pointcloud.");

  return 0;
}
