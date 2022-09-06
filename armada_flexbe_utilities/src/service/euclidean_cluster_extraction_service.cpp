/*
 * Nothing here yet
 */

#include "ros/ros.h"
#include "armada_flexbe_utilities/EuclideanClusterExtraction.h"

using namespace std;

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

    // Do stuff here

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
