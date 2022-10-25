#include "ros/ros.h"
#include "armada_flexbe_utilities/EuclideanClusterExtraction.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;


class EuclideanClusterExtractionService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer euclideanClusterExtractionService;
  double clusterTolerance;
  int minClusterSize;
  int maxClusterSize;

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
   * Segment clusters within a PointCloud into individual cloud objects.
   *
   * BGiven a PointCloud2 message, segment clusters of points into their own PointCloud2 objects for further processing/handling.
   *
   * @param[in] req Fill in the type of data and its general purpose.
   * @param[out] res Fill in the type of data and its general purpose.
   * @return Bool Service completion result.
   */
  bool euclideanClusterExtraction(armada_flexbe_utilities::EuclideanClusterExtraction::Request &req,
                                  armada_flexbe_utilities::EuclideanClusterExtraction::Response &res)
  {
    nh_.getParam("/filters/euclidean_cluster_extraction/clusterTolerance", clusterTolerance);
    nh_.getParam("/filters/euclidean_cluster_extraction/minClusterSize", minClusterSize);
    nh_.getParam("/filters/euclidean_cluster_extraction/maxClusterSize", maxClusterSize);

    PointCloud<PointXYZRGB>::Ptr input_cloud (new PointCloud<PointXYZRGB>);
    sensor_msgs::PointCloud2::Ptr temp_cloud (new sensor_msgs::PointCloud2);
    std::vector<sensor_msgs::PointCloud2> cluster_cloud_list;
    fromROSMsg(req.cloud_in, *input_cloud);

    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
    tree->setInputCloud (input_cloud);

    EuclideanClusterExtraction<PointXYZRGB> ec;
    std::vector<PointIndices> cluster_indices;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
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
  ros::spin();

  return 0;
}
