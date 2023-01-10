#include <ros/ros.h>
#include "armada_flexbe_utilities/GetPointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

class GetPointCloudService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer getPointCloudService;
  std::string camera_topic;

public:

  /**
   * Class Constructor.;
   *
   * Constructor for GetPointCloudService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  GetPointCloudService(ros::NodeHandle nh) :
    nh_(nh)
  {
    getPointCloudService = nh.advertiseService("get_pointcloud", &GetPointCloudService::getPointCloud, this);
  }


  /**
   * Add current camera pointcloud msg to array of pointcloud messages.
   *
   * Given a camera topic, listen for a PointCloud2 message and transform it into the base_link frame for further processing.
   *
   * @param[in] req string String containing desired camera message topic.
   * @param[out] res sensor_msgs/PointCloud2 PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool getPointCloud(armada_flexbe_utilities::GetPointCloud::Request &req,
                     armada_flexbe_utilities::GetPointCloud::Response &res)
  {
    nh_.getParam("/camera_topic", camera_topic);

    tf::StampedTransform transform;
    tf::TransformListener listener;

    ros::Duration timeout(3);
    sensor_msgs::PointCloud2ConstPtr pointcloud2_msg_dummy = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic, timeout);
    sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic, timeout);
    ros::Time stamp = ros::Time(0);

    pcl::PointCloud<pcl::PointXYZRGB> temp_transform_cloud;
    fromROSMsg(*pointcloud2_msg, temp_transform_cloud);

    pcl_conversions::toPCL(stamp, temp_transform_cloud.header.stamp);

    try
    {
      listener.waitForTransform("base_link", temp_transform_cloud.header.frame_id, stamp, timeout);
      listener.lookupTransform("base_link", temp_transform_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
      ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("base_link", temp_transform_cloud, temp_transform_cloud, listener);

    sensor_msgs::PointCloud2 transformed_pointcloud_msg;
    toROSMsg(temp_transform_cloud, transformed_pointcloud_msg);

    res.cloud_out = transformed_pointcloud_msg;
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pointcloud_service");
  ros::NodeHandle nh;

  GetPointCloudService getPointCloudService = GetPointCloudService(nh);
  ros::spin();

  return 0;
}
