#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from armada_flexbe_utilities.srv import GenGraspWaypoints
from geometry_msgs.msg import Pose


class graspWaypointsServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- target_time 	float 	Time which needs to have passed since the behavior started.

        #> x            int             random x position.
        #> y            int             random y position.

        <= continue 			generated x,y position.
        <= failed 			something went wrong.

        '''

        def __init__(self, grasp_offset, pregrasp_dist, postgrasp_dist):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(graspWaypointsServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['grasp_msg_list'],
                                                        output_keys = ['grasp_poses_list'])

                # store object spawn pose info from previous state
                self._grasp_offset = grasp_offset
                self._pregrasp_dist = pregrasp_dist
                self._postgrasp_dist = postgrasp_dist

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                # check/wait for spawn model service to spin up
                rospy.wait_for_service('gen_grasp_waypoints')
                grasp_waypoints_srv = rospy.ServiceProxy('gen_grasp_waypoints', GenWaypoints)

                try:
                  grasp_waypoints_srv(userdata.grasp_msg_list, self._grasp_offset, self._pregrasp_dist, self._postgrasp_dist)
                  userdata.grasp_poses_list = grasp_waypoints_srv.grasp_poses_list
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to generate a list of possible grasp waypoints...' )

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                pass # Nothing to do in this state.

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.

