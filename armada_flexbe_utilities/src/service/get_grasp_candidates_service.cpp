#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include "armada_flexbe_utilities/GetGraspCandidates.h"
#include <sensor_msgs/PointCloud2.h>
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>

class GetGraspCandidateService
{
protected:

  ros::NodeHandle nh_;
  ros::ServiceServer getGraspCandidateService;
  int timeout_len;

public:

  /**
   * Class Constructor.
   *
   * Constructor for SetGripperService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  GetGraspCandidateService(ros::NodeHandle nh) :
    nh_(nh)
  {
    getGraspCandidateService = nh.advertiseService("get_grasp_candidates", &GetGraspCandidateService::GetGraspCandidates, this);
  }

  /**
   * Get a list of grasp candidates from active grasp planning algorithm.
   *
   * Given a topic to publish a pointcloud on, topic to listen for grasp candidates, and a PointCloud2 message,
   * get a list of grasp candidates for the grasp target from the active grasp planning algorithm.
   *
   * @param[in] req string String containing topic the grasp planner listens for PointCloud2 messages on.
   * @param[in] req string String containing topic the grasp planner sends grasp candidate messages on.
   * @param[in] req sensor_msgs/PointCloud2 PointCloud2 grasp target PointCloud2 message.
   * @param[out] res gpd_ros/GraspConfigList message containing list of grasp candidates.
   * @return Bool Service completion result.
   */
  bool GetGraspCandidates(armada_flexbe_utilities::GetGraspCandidates::Request &req,
                          armada_flexbe_utilities::GetGraspCandidates::Response &res)
  {
    nh_.getParam("/grasp/timeout_len", timeout_len);

    ros::Duration timeout(timeout_len);
    boost::shared_ptr<gpd_ros::GraspConfigList const> sharedGraspConfigListPtr;
    sharedGraspConfigListPtr = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(req.grasp_candidates_topic, timeout);
    if(sharedGraspConfigListPtr != NULL) {
      res.grasp_msg_list = *sharedGraspConfigListPtr;
    }

    if(res.grasp_msg_list.grasps.size() == 0){
      return false;
    }
    else {
      return true;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_grasp_candidates_service");
  ros::NodeHandle nh;

  GetGraspCandidateService getGraspCandidateService = GetGraspCandidateService(nh);
  ros::spin();

  return 0;
}
