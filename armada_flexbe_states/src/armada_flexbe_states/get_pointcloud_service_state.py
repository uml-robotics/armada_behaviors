#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from sensor_msgs.msg import PointCloud2

from armada_flexbe_utilities.srv import GetPointCloud, GetPointCloudResponse, GetPointCloudRequest
from armada_flexbe_utilities.srv import PCLVoxelGridFilter, PCLVoxelGridFilterResponse, PCLVoxelGridFilterRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

class GetPointCloudServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- camera_topic                 string          The desired camera_topic

        ># pointcloud_list                              List of PointCloud2 messages
        #> pointcloud_list                              List of PointCloud2 messages

        <= continue                                     retrieved the pointcloud successfully
        <= failed                                       something went wrong

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(GetPointCloudServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_list'],
                                                       output_keys = ['pointcloud_list'])

                self._service_topic = '/get_pointcloud'
                self._service = ProxyServiceCaller({self._service_topic: GetPointCloud})

                self._service_topic_two = '/voxelgrid_filter'
                self._service_two = ProxyServiceCaller({self._service_topic_two: PCLVoxelGridFilter})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                response = GetPointCloudResponse()

                try:
                    get_response = self._service.call(self._service_topic, Empty)
                    filter_response = self._service.call(self._service_topic_two, get_response.cloud_out)
                    userdata.pointcloud_list.append(filter_response.cloud_out)
                    return "continue"
                except:
                    return "failed"

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('getting pointcloud snapshot...' )

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                rospy.wait_for_service(self._service_topic)
                rospy.wait_for_service(self._service_topic_two)

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.

