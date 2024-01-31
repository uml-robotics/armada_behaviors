#include <ros/ros.h>
#include "armada_flexbe_utilities/MoveArmCartesian.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class MoveArmCartesianService
{
protected:

    ros::NodeHandle nh_;
    ros::ServiceServer moveArmCartesianService;
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
    MoveArmCartesianService(ros::NodeHandle nh) :
        nh_(nh)
    {
        nh.getParam("/move_group/planning_group", planning_group_);
        MoveGroupPtr_ = MoveGroupPtr(new moveit::planning_interface::MoveGroupInterface(planning_group_));
        PlanningScenePtr_ = PlanningScenePtr(new moveit::planning_interface::PlanningSceneInterface);

        moveArmCartesianService = nh.advertiseService("move_arm_cartesian", &MoveArmCartesianService::moveArmCartesian, this);
    }

    /**
    * Helper function
    * Plan a cartesian path.
    *
    * Plan a cartesian path along one or more set points using the MoveIt interface.
    *
    * @param[in] pose_list Container whose values make up points along the intended path.
    * @param[out] my_plan MoveIt path plan which was solved for in this function.
    * @return Percent (value from 0->100) of points along path between given points successfully planned.
    */
    double cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
    {
        nh_.getParam("/move_group/jump_threshold", jump_threshold_);
        nh_.getParam("/move_group/eef_step", eef_step_);

        moveit_msgs::RobotTrajectory trajectory;
        double success = MoveGroupPtr_->computeCartesianPath(pose_list, eef_step_, jump_threshold_, trajectory);
        success *= 100;
        my_plan.trajectory_ = trajectory;
        ros::Duration(0.5).sleep();   // this was in here previously, testing to see if necessary
        return success;
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
    bool moveArmCartesian(armada_flexbe_utilities::MoveArmCartesian::Request &req,
                          armada_flexbe_utilities::MoveArmCartesian::Response &res)
    {
        std::vector<geometry_msgs::Pose> waypoints;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        waypoints.insert(waypoints.begin(), std::begin(req.grasp_waypoints), std::end(req.grasp_waypoints));

        res.plan_success = cartesianPlan(waypoints, plan);

        if (res.plan_success >= 100) {
            MoveGroupPtr_->execute(plan);
            ros::Duration(0.5).sleep();
            return true;
        } else {
            return false;
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_arm_cartesian_service");
    ros::NodeHandle nh;

    MoveArmCartesianService moveArmCartesianService = MoveArmCartesianService(nh);
    ros::spin();

    return 0;
}