#include <ros/ros.h>
#include "armada_flexbe_utilities/VoxelGridFilter.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;

class VoxelGridFilterService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer voxelGridFilterService;
  float leaf_size_x;
  float leaf_size_y;
  float leaf_size_z;

public:

  /**
   * Class Constructor.
   *
   * Constructor for VoxelGridFilterService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  VoxelGridFilterService(ros::NodeHandle nh) :
    nh_(nh)
  {
    voxelGridFilterService = nh.advertiseService("voxelgrid_filter", &VoxelGridFilterService::voxelGridFilter, this);
  }

  /**
   * Downsample a PointCloud2 message by applying a voxelgrid filter.
   *
   * Given a PointCloud2 message, apply a voxelgrid filter and provide the resulting PointCloud2 message.
   * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
   * This filter: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool voxelGridFilter(armada_flexbe_utilities::VoxelGridFilter::Request &req,
                       armada_flexbe_utilities::VoxelGridFilter::Response &res)
  {
    PointCloud<PointXYZRGB>::Ptr input_cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *input_cloud);

    nh_.getParam("/filters/voxel_grid/leaf_size_x", leaf_size_x);
    nh_.getParam("/filters/voxel_grid/leaf_size_y", leaf_size_y);
    nh_.getParam("/filters/voxel_grid/leaf_size_z", leaf_size_z);

    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud (input_cloud);
    vox.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
    vox.filter (*filtered_cloud);

    toROSMsg(*filtered_cloud, res.cloud_out);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "voxel_grid_filter_service");
  ros::NodeHandle nh;

  VoxelGridFilterService voxelGridFilterService = VoxelGridFilterService(nh);
  ros::spin();

  return 0;
}
