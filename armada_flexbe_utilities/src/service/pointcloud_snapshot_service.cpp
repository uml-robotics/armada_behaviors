#include <ros/ros.h>
#include "armada_flexbe_utilities/GetPointCloud.h"
#include "armada_flexbe_utilities/ConcatenatePointCloud.h"
#include "armada_flexbe_utilities/SacSegmentation.h"
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

class PointCloudSnapshotService
{
protected:

  ros::ServiceServer getPointCloudService;
  ros::ServiceServer concatenatePointCloudService;
  ros::ServiceServer sacSegmentationService;
  ros::ServiceServer passthroughFilterService;

public:

  /**
   * Class Constructor.
   *
   * Constructor for PointCloudSnapshotService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  PointCloudSnapshotService(ros::NodeHandle nh)
  {
    getPointCloudService = nh.advertiseService("get_pointcloud", &PointCloudSnapshotService::getPointCloud, this);
    concatenatePointCloudService = nh.advertiseService("concatenate_pointcloud", &PointCloudSnapshotService::concatenatePointCloud, this);
    sacSegmentationService = nh.advertiseService("sac_segmentation", &PointCloudSnapshotService::sacSegmentation, this);
    passthroughFilterService = nh.advertiseService("passthrough_filter", &PointCloudSnapshotService::passthroughFilter, this);
  }

  /**
   * Class Destructor.
   *
   * Destructor for PointCloudSnapshotService class.
   */
  ~PointCloudSnapshotService(void)
  {
  }

  /**
   * Add current camera pointcloud msg to array of pointcloud messages.
   *
   * Given a topic and existing array of PointCloud2 messages, add another PointCloud2 message from the given topic to the array of messages.
   *
   * @param[in] req string String containing desired camera message topic.
   * @param[out] res sensor_msgs/PointCloud2 PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool getPointCloud(armada_flexbe_utilities::GetPointCloud::Request &req,
                     armada_flexbe_utilities::GetPointCloud::Response &res)
  {
    ROS_WARN("Executing GetPointCloud Service");
    ros::Duration timeout(10);
    sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.camera_topic, timeout);

    PointCloud<PointXYZRGB> transformed_cloud;
    transformed_cloud = transformCloud(*pointcloud2_msg, "world");

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    toROSMsg(transformed_cloud, transformed_cloud_msg);

    res.cloud_out = transformed_cloud_msg;
    ROS_WARN("Finished GetPointCloud Service");
    return true;
  }

  /**
   * Concatenate an array of PointCloud2 into a single PointCloud2.
   *
   * Given an array of PointCloud2 messages, concatenate into a single PointCloud2 message for publishing.
   *
   * @param[in] req sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool concatenatePointCloud(armada_flexbe_utilities::ConcatenatePointCloud::Request &req,
                             armada_flexbe_utilities::ConcatenatePointCloud::Response &res)
  {
    ROS_WARN("Executing ConcatenatePointCloud Service");
    unsigned int cloud_list_size = req.cloud_list_in.size();
    ROS_WARN("PointCloud2 List Size: %d ...", cloud_list_size);
    PointCloud<PointXYZRGB> cloud_array[cloud_list_size];
    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);

    *temp_cloud = cloud_array[0];

    if (cloud_list_size > 1) {
      for (unsigned int i = 1; i < cloud_list_size; i++) {
        *temp_cloud+= cloud_array[i];
        ROS_WARN("Concatenating cloud number: %d ...", i);
      }
    }

    ROS_WARN("Finishing ConcatenatePointCloud Service");
    toROSMsg(*temp_cloud, res.cloud_out);

    return true;
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
  ros::init(argc, argv, "pointcloud_processing_service");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  PointCloudSnapshotService pointCloudSnapShotService = PointCloudSnapshotService(nh);

  while(ros::ok())
  {
    // spin until shutdown
  }

  return 0;
}
