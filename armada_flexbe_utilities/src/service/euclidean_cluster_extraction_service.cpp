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
    ROS_WARN("Executing EuclideanClusterExtraction Service");
    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr desired_cluster (new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *temp_cloud);


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
     VoxelGrid<PointXYZRGB> vg;
     PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
     vg.setInputCloud (temp_cloud);
     vg.setLeafSize (0.01f, 0.01f, 0.01f);
     vg.filter (*cloud_filtered);


     // Creating the KdTree object for the search method of the extraction
     search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
     tree->setInputCloud (cloud_filtered);

     std::vector<PointIndices> cluster_indices;
     EuclideanClusterExtraction<PointXYZRGB> ec;
     ec.setClusterTolerance (0.02); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_filtered);
     ec.extract (cluster_indices);

     unsigned long j = 0;
     int biggest_size = 0;
     for (std::vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       PointCloud<PointXYZRGB>::Ptr cloud_cluster (new PointCloud<PointXYZRGB>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)

       cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
       cloud_cluster->width = cloud_cluster->size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;
       cloud_cluster->header.frame_id = cloud_filtered->header.frame_id;

       int cluster_size = cloud_cluster->size();

       if (biggest_size < cluster_size)
       {
        biggest_size = cluster_size;
        desired_cluster = cloud_cluster;
       }
     }

     toROSMsg(*desired_cluster, res.cluster_cloud);

    ROS_WARN("Finished EuclideanClusterExtraction Service");
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
