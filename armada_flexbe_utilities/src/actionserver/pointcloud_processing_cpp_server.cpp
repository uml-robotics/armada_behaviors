#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <armada_flexbe_utilities/PointcloudProcessingAction.h>
#include <tf/transform_listener.h>

typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

class PointcloudProcessingCPPAction
{
protected:
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<armada_flexbe_utilities::PointcloudProcessingAction> CapturePointcloudServer_;
  armada_flexbe_utilities::PointcloudProcessingFeedback capture_pointcloud_feedback_;
  armada_flexbe_utilities::PointcloudProcessingResult capture_pointcloud_result_;

public:

  /**
   * Class Constructor.
   *
   * Constructor for PointcloudProcessingCPPAction class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  PointcloudProcessingCPPAction(ros::NodeHandle nh) :
    CapturePointcloudServer_(nh, "capture_pointcloud", boost::bind(&PointcloudProcessingCPPAction::capturePointcloud, this, _1), false)
  {
    CapturePointcloudServer_.start();
  }

  /**
   * Class Destructor.
   *
   * Destructor for PointcloudProcessingCPPAction class.
   */
  ~PointcloudProcessingCPPAction(void)
  {
  }

  /**
   * Plan a cartesian path.
   *
   * Plan a cartesian path along one or more set points using the MoveIt interface.
   *
   * @param[in] pose_list Container whose values make up points along the intended path.
   * @param[out] my_plan MoveIt path plan which was solved for in this function.
   * @return Percent (value from 0->100) of points along path between given points successfully planned.
   */
  void capturePointcloud(const armada_flexbe_utilities::PointcloudProcessingGoalConstPtr &goal)
  {
    //return success;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_processing_cpp_server");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  PointcloudProcessingCPPAction pointcloud_processing_cpp_server(nh);

  while(ros::ok())
  {
    // spin until shutdown
  }

  return 0;
}
