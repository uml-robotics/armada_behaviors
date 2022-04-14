#include "ros/ros.h"
#include "armada_flexbe_utilities/SetGripperVal.h"
#include <control_msgs/GripperCommandActionGoal.h>

using namespace std;

class SetGripperService
{
protected:

  ros::ServiceServer setGripperService;
  ros::Publisher gripper_cmd;

public:

  /**
   * Class Constructor.
   *
   * Constructor for SetGripperService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  SetGripperService(ros::NodeHandle nh)
  {
    setGripperService = nh.advertiseService("set_gripper", &SetGripperService::setGripper, this);
  }

  /**
   * Class Destructor.
   *
   * Destructor for SetGripperService class.
   */
  ~SetGripperService(void)
  {
  }

  /**
   * Add current camera pointcloud msg to array of pointcloud messages.
   *
   * Given a topic and existing array of PointCloud2 messages, add another PointCloud2 message from the given topic to the array of messages.
   *
   * @param[in] req sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
   * @param[out] res sensor_msgs/PointCloud2[] Container of PointCloud2 messages.
   * @return Bool Service completion result.
   */
  bool setGripper(armada_flexbe_utilities::SetGripperVal::Request &req,
                  armada_flexbe_utilities::SetGripperVal::Response &res)
  {
    ros::NodeHandle temp_nh;
    string gripper_topic = req.gripper_namespace + "/" + req.gripper_name + "_gripper_controller/gripper_cmd/goal";
    gripper_cmd = temp_nh.advertise<control_msgs::GripperCommandActionGoal>(gripper_topic, 10);
    ros::Duration gripper_delay(req.wait_time);
    res.success = true;

    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_pointcloud_service");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  SetGripperService setGripperService = SetGripperService(nh);

  while(ros::ok())
  {
    // spin until shutdown
  }

  return 0;
}
