#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/voxel_grid.h>
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
    pcl::PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    fromROSMsg(req.cloud_in, *temp_cloud);

    // Read in the cloud data
//    pcl::PCDReader reader;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//    reader.read ("/home/csrobot/Downloads/table_scene_lms400.pcd", *cloud); //give this a real file path
//    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//     pcl::VoxelGrid<pcl::PointXYZRGB> vg;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//     vg.setInputCloud (temp_cloud);
//     vg.setLeafSize (0.01f, 0.01f, 0.01f);
//     vg.filter (*cloud_filtered);
//     std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

//     // Create the segmentation object for the planar model and set all the parameters
//     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
//     pcl::PCDWriter writer;
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setMaxIterations (100);
//     seg.setDistanceThreshold (0.02);

     int i=0, nr_points = (int) temp_cloud->size ();
     while (temp_cloud->size () > 0.3 * nr_points)
     {
//       // Segment the largest planar component from the remaining cloud
//       seg.setInputCloud (cloud_filtered);
//       seg.segment (*inliers, *coefficients);
//       if (inliers->indices.size () == 0)
//       {
//         std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//         break;
//       }

       // Extract the planar inliers from the input cloud
       pcl::ExtractIndices<pcl::PointXYZRGB> extract;
       extract.setInputCloud (temp_cloud);
//       extract.setIndices (inliers);
       extract.setNegative (false);

       // Get the points associated with the planar surface
       extract.filter (*temp_cloud);
       std::cout << "PointCloud representing the planar component: " << temp_cloud->size () << " data points." << std::endl;

       // Remove the planar inliers, extract the rest
//       extract.setNegative (true);
//       extract.filter (*cloud_filtered);
//       *cloud_filtered = *cloud_filtered;
     }

     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
     tree->setInputCloud (temp_cloud);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
     ec.setClusterTolerance (0.02); // 2cm
     ec.setMinClusterSize (100);
     ec.setMaxClusterSize (25000);
     ec.setSearchMethod (tree);
     ec.setInputCloud (temp_cloud);
     ec.extract (cluster_indices);

     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
       for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
         cloud_cluster->push_back ((*temp_cloud)[*pit]); //*
       cloud_cluster->width = cloud_cluster->size ();
       cloud_cluster->height = 1;
       cloud_cluster->is_dense = true;

//       std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
//       std::stringstream ss;
//       ss << "cloud_cluster_" << j << ".pcd";
//       writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
//       j++;
//        toROSMsg(cloud_cluster, res.cloud_out);
       // string topic_name = ss;
       // ros::Publisher<sensor_msgs::PointCloud2>(topic_name, 10); <- wrong
       toROSMsg(*temp_cloud, res.cloud_list_out[j]);
     }

    //toROSMsg(*temp_cloud, res.cloud_out);
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
