#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from armada_flexbe_utilities.srv import GPDGraspWaypoints, GPDGraspWaypointsResponse, GPDGraspWaypointsRequest


class CalculateGraspWaypointsServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- grasp_offset                                 Topic to publish pointcloud message
        -- pregrasp_dist                                Topic to subscribe for grasp candidate messages
        -- postgrasp_dist                               Topic to subscribe for grasp candidate messages

        ># grasp_candidates                             List of grasp candidates message
        #> grasp_waypoints_list                         List of sets of grasp waypoints

        <= continue                                     Calculated list of sets of grasp waypoints
        <= failed                                       Something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(CalculateGraspWaypointsServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['grasp_candidates'],
                                                        output_keys = ['grasp_waypoints_list'])

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                self._service_topic = '/calculate_grasp_waypoints'
                rospy.wait_for_service(self._service_topic)
                self._service = ProxyServiceCaller({self._service_topic: GPDGraspWaypoints})

                request = GPDGraspWaypointsRequest()
                request.grasp_msg_list = userdata.grasp_candidates

                try:
                  service_response = self._service.call(self._service_topic, request)
                  userdata.grasp_waypoints_list = service_response.grasp_poses_list.poses
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to generate a list of grasp waypoint sets...' )

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

