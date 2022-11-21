#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from armada_flexbe_utilities.srv import PCLRadiusOutlierRemoval, PCLRadiusOutlierRemovalResponse, PCLRadiusOutlierRemovalRequest


class PCLRadiusOutlierRemovalServiceState(EventState):
        '''
        Remove all indices from within a PointCloud that do not have at least the specified number of neighbors within a defined range (values defined within service).

        ># pointcloud_in                                Unfiltered PointCloud2 message
        #> pointcloud_out                               Filtered PointCloud2 message

        <= continue                                     Filtered pointcloud successfully
        <= failed                                       Something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(PCLRadiusOutlierRemovalServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_in'],
                                                       output_keys = ['pointcloud_out'])

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                self._service_topic = '/radius_outlier_removal'
                rospy.wait_for_service(self._service_topic)
                self._service = ProxyServiceCaller({self._service_topic: PCLRadiusOutlierRemoval})

                try:
                  service_response = self._service.call(self._service_topic, userdata.pointcloud_in)
                  userdata.pointcloud_out = service_response.cloud_out
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to segment planes from pointcloud...' )

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

