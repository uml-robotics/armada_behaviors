#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from armada_flexbe_utilities.srv import PointCloudPassthroughFilter, PointCloudPassthroughFilterResponse, PointCloudPassthroughFilterRequest


class PCLPassthroughFilterServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- x_min                   float32              Desired X lower limit for pointcloud filtering
        -- x_max                   float32              Desired X upper limit for pointcloud filtering
        -- y_min                   float32              Desired Y lower limit for pointcloud filtering
        -- y_max                   float32              Desired Y upper limit for pointcloud filtering
        -- z_min                   float32              Desired Z lower limit for pointcloud filtering
        -- z_max                   float32              Desired Z upper limit for pointcloud filtering

        ># pointcloud_in                                Unfiltered PointCloud2 message
        #> pointcloud_out                               Filtered PointCloud2 message

        <= continue                                     Filtered pointcloud successfully
        <= failed                                       Something went wrong

        '''

        def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(PCLPassthroughFilterServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_in'],
                                                       output_keys = ['pointcloud_out'])

                self._x_min = x_min
                self._x_max = x_max
                self._y_min = y_min
                self._y_max = y_max
                self._z_min = z_min
                self._z_max = z_max

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                self._service_topic = '/passthrough_filter'
                rospy.wait_for_service(self._service_topic)
                self._service = ProxyServiceCaller({self._service_topic: PointCloudPassthroughFilter})

                request = PointCloudPassthroughFilterRequest()
                request.cloud_in = userdata.pointcloud_in
                request.x_min = self._x_min
                request.x_max = self._x_max
                request.y_min = self._y_min
                request.y_max = self._y_max
                request.z_min = self._z_min
                request.z_max = self._z_max

                try:
                  service_response = self._service.call(self._service_topic, request)
                  userdata.pointcloud_out = service_response.cloud_out
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to filter pointcloud...' )

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

