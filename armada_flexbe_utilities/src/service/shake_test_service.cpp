#include <ros/ros.h>
#include "armada_flexbe_utilities/ShakeTest.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class ShakeTestService
{
protected:

    ros::NodeHandle nh_;
    ros::ServiceServer shakeTestService;
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
    ShakeTestService(ros::NodeHandle nh) :
        nh_(nh)
    {
        nh.getParam("/move_group/planning_group", planning_group_);
        MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));
        PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);

        shakeTestService = nh.advertiseService("shake_test", &ShakeTestService::shakeTest, this);
    }

    /**
     * Perform the WPI benchmarking shake test
     * 
     * Rotate certain links to predefined positions
     *
     * @param[in] req Empty.
     * @param[out] res bool success.
     * @return Bool Service completion result.
     */
    bool shakeTest(armada_flexbe_utilities::ShakeTest::Request &req,
                   armada_flexbe_utilities::ShakeTest::Response &res)
    {
        // Rotate the wrist_3_joint
        MoveGroupPtr_->setJointValueTarget("wrist_3_joint", 0.785398);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_3_joint", -0.785398);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_3_joint", 0.0);
        MoveGroupPtr_->move();

        // Rotate the wrist_2_joint
        MoveGroupPtr_->setJointValueTarget("wrist_2_joint", 2.35619);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_2_joint", 0.785398);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_2_joint", 2.35619);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_2_joint", 0.785398);
        MoveGroupPtr_->move();

        MoveGroupPtr_->setJointValueTarget("wrist_2_joint", 1.5708);
        MoveGroupPtr_->move();

        res.success = true;
        return true;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shake_test_service");
    ros::NodeHandle nh;

    ShakeTestService shakeTestService = ShakeTestService(nh);
    ros::spin();

    return 0;
}