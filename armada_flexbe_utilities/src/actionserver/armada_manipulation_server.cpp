#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <armada_flexbe_utilities/ArmadaManipulationAction.h>
//#include <armada_flexbe_utilities/ArmadaManipulationClass.h>

typedef actionlib::SimpleActionServer<armada_flexbe_utilities::ArmadaManipulationAction> Server1;
typedef actionlib::SimpleActionServer<armada_flexbe_utilities::ArmadaManipulationAction> Server2;

void execute1(const armada_flexbe_utilities::ArmadaManipulationGoalConstPtr& goal, Server1* actionServer)
{
  actionServer->setSucceeded();
}

void execute2(const armada_flexbe_utilities::ArmadaManipulationGoalConstPtr& goal, Server2* actionServer)
{
  actionServer->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armada_manipulation_server");
  ros::NodeHandle nh;

  // Start servers
  Server1 server1(n, "move1", boost::bind(&execute1, _1, &server), false);
  server1.start();
  Server2 server2(n, "move2", boost::bind(&execute2, _1, &server), false);
  server2.start();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  return 0;
}
