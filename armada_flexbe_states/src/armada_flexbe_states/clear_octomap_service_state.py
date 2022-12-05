#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class ClearOctomapServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        <= continue                                     cleared octomap successfully
        <= failed                                       something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(ClearOctomapServiceState, self).__init__(outcomes = ['continue', 'failed'])

                self._service_topic = '/clear_octomap'
                self._service = ProxyServiceCaller({self._service_topic: Empty})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                  #service_response = self._service.call(self._service_topic, )
                  clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
                  clear_octomap()
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to clear octomap...' )

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

