#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from armada_flexbe_utilities.srv import PCLEuclideanClusterExtraction, PCLEuclideanClusterExtractionResponse, PCLEuclideanClusterExtractionRequest


class PCLEuclideanClusterExtractionServiceState(EventState):
        '''
        Extract Euclidean clusters from within a given PointCloud.

        -- param                        string          Param description

        ># input_key                                    Input Key info
        #> output_key                                   Output Key info

        <= continue                                     Continue outcome description
        <= failed                                       Failure outcome description

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(PCLEuclideanClusterExtractionServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_in', 'obstacles_cloud_list_in'],
                                                       output_keys = ['target_cloud_out', 'obstacles_cloud_list_out'])

                self._service_topic = '/euclidean_cluster_extraction'
                self._service = ProxyServiceCaller({self._service_topic: PCLEuclideanClusterExtraction})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                  service_response = self._service.call(self._service_topic, userdata.pointcloud_in)
                  userdata.target_cloud_out = service_response.target_cloud_out
                  temp_cloud_list = []
                  temp_cloud_list.extend(userdata.obstacles_cloud_list_in)
                  temp_cloud_list.extend(service_response.obstacle_cloud_list_out)
                  userdata.obstacles_cloud_list_out = temp_cloud_list
                  return 'continue'
                except:
                  return 'failed'


                pass # Add functionality here if necessary

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to extract object clusters...' )

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Add functionality here if necessary

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                rospy.wait_for_service(self._service_topic)

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Add functionality here if necessary

