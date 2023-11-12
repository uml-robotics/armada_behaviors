#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from sensor_msgs.msg import PointCloud2

from armada_flexbe_utilities.srv import GPDGraspCandidates, GPDGraspCandidatesResponse, GPDGraspCandidatesRequest

class GPDGraspCandidatesServiceState(EventState):
        '''
        TODO

        ># combined_pointcloud                          List of PointCloud2 message
        #> grasp_candidates                             List of grasp candidates message

        <= continue                                     Retrieved grasp candidates
        <= failed                                       Something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(GPDGraspCandidatesServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['combined_pointcloud'],
                                                       output_keys = ['grasp_candidates'])

                self._service_topic = '/gpd_grasp_candidates'
                self._service = ProxyServiceCaller({self._service_topic: GPDGraspCandidates})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                  service_response = self._service.call(self._service_topic, userdata.combined_pointcloud)
                  userdata.grasp_candidates = service_response.grasp_msg_list
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to get grasp candidates...' )

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                rospy.wait_for_service(self._service_topic)

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.