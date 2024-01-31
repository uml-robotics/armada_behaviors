#include <ros/ros.h>
#include "armada_flexbe_utilities/MoveArmNamedPose.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class MoveArmNamedPoseService
{
protected:

    ros::NodeHandle nh_;
    ros::ServiceServer moveArmNamedPoseService;
    std::string planning_group_;
    MoveGroupPtr MoveGroupPtr_;
    PlanningScenePtr PlanningScenePtr_;
    double jump_threshold_;
    double eef_step_;

public:

    /**
    * Class Constructor.
    *
    * Constructor for MoveArmCartesianService class.
    *
    * @param[in] nh A ROS NodeHandle object.
    */
    MoveArmNamedPoseService(ros::NodeHandle nh) :
        nh_(nh)
    {
        nh.getParam("/move_group/planning_group", planning_group_);
        MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));
        PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);

        moveArmNamedPoseService = nh.advertiseService("move_arm_named_pose", &MoveArmNamedPoseService::moveArmNamedPose, this);
    }

    /**
     * Plan a cartesian path.
     *
     * Plan a cartesian path along one or more set points using the MoveIt interface.
     *
     * @param[in] req Container whose values make up points along the intended path.
     * @param[out] res float64 cartesian plan success rate for accepted plan.
     * @return Bool Service completion result.
     */
    bool moveArmNamedPose(armada_flexbe_utilities::MoveArmNamedPose::Request &req,
                          armada_flexbe_utilities::MoveArmNamedPose::Response &res)
    {
        std::vector<std::string> poses;
        poses.insert(poses.begin(), std::begin(req.pose_names), std::end(req.pose_names));

        unsigned long n = poses.size();
        for (unsigned long i = 0; i < n; i++) {
            MoveGroupPtr_->setNamedTarget(poses[i]);
            MoveGroupPtr_->move();
        }
        res.plan_success = 1;
        return true;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_arm_named_pose_service");
    ros::NodeHandle nh;

    MoveArmNamedPoseService moveArmNamedPoseService = MoveArmNamedPoseService(nh);
    ros::spin();

  return 0;
}