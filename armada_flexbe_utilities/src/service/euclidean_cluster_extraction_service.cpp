#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "ros/ros.h"
#include "armada_flexbe_utilities/EuclideanClusterExtraction.h"

using namespace pcl;


class EuclideanClusterExtractionService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer euclideanClusterExtractionService;

public:

  /**
   * Class Constructor.
   *
   * Constructor for EuclideanClusterExtractionService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  EuclideanClusterExtractionService(ros::NodeHandle nh) :
    nh_(nh)
  {
    euclideanClusterExtractionService = nh.advertiseService("euclidean_cluster_extraction", &EuclideanClusterExtractionService::euclideanClusterExtraction, this);
  }

  /**
   * Brief overview description of function usage.
   *
   * Brief detail of what the function does (e.g., Takes in a type of data, does a process and returns some other data).
   *
   * @param[in] req Fill in the type of data and its general purpose.
   * @param[out] res Fill in the type of data and its general purpose.
   * @return Bool Service completion result.
   */
  bool euclideanClusterExtraction(armada_flexbe_utilities::EuclideanClusterExtraction::Request &req,
                                  armada_flexbe_utilities::EuclideanClusterExtraction::Response &res)
  {
    PointCloud<PointXYZRGB>::Ptr input_cloud (new PointCloud<PointXYZRGB>);
    sensor_msgs::PointCloud2::Ptr temp_cloud (new sensor_msgs::PointCloud2);
    std::vector<sensor_msgs::PointCloud2> cluster_cloud_list;
    fromROSMsg(req.cloud_in, *input_cloud);

    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
    tree->setInputCloud (input_cloud);

    EuclideanClusterExtraction<PointXYZRGB> ec;
    std::vector<PointIndices> cluster_indices;
    ec.setClusterTolerance (0.02);
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (cluster_indices);

    for (std::vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      PointCloud<PointXYZRGB>::Ptr cloud_cluster (new PointCloud<PointXYZRGB>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
        cloud_cluster->push_back ((*input_cloud)[*pit]);
      }

      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_cluster->header.frame_id = input_cloud->header.frame_id;

      toROSMsg(*cloud_cluster, *temp_cloud);
      cluster_cloud_list.push_back(*temp_cloud);
    }

    // change srv res
    res.cluster_cloud_list = cluster_cloud_list;
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "euclidean_cluster_extraction_service");
  ros::NodeHandle nh;

  EuclideanClusterExtractionService euclideanClusterExtractionService = EuclideanClusterExtractionService(nh);
  ROS_WARN("euclidean_cluster_extraction_service Ready.");
  ros::spin();

  return 0;
}
