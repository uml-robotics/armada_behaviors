#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from armada_flexbe_utilities.srv import *


class concatenatePointCloudState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- x_min                   float64              Desired X lower limit for pointcloud filtering
        -- x_max                   float64              Desired X upper limit for pointcloud filtering
        -- y_min                   float64              Desired Y lower limit for pointcloud filtering
        -- y_max                   float64              Desired Y upper limit for pointcloud filtering
        -- z_min                   float64              Desired Z lower limit for pointcloud filtering
        -- z_max                   float64              Desired Z upper limit for pointcloud filtering

        ># pointcloud_list                              List of PointCloud2 messages
        #> combined_pointcloud                          Filtered & concatenated PointCloud2 message

        <= continue                                     spawned/deleted an object successfully
        <= failed                                       something went wrong

        '''

        def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(concatenatePointCloudState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_list'],
                                                       output_keys = ['combined_pointcloud'])

                rospy.wait_for_service('/concatenate_pointcloud')
                rospy.wait_for_service('/passthrough_filter')
                rospy.wait_for_service('/sac_segmentation')

                self._concatenate_pointcloud_srv_topic = '/concatenate_pointcloud'
                self._concatenate_pointcloud_srv = ProxyServiceCaller({self._concatenate_pointcloud_srv_topic: ConcatenatePointCloud})

                self._passthrough_filter_srv_topic = '/passthrough_filter'
                self._passthrough_filter_srv = ProxyServiceCaller({self._passthrough_filter_srv_topic: PointCloudPassthroughFilter})

                self._sac_segmentation_srv_topic = '/sac_segmentation'
                self._sac_segmentation_srv = ProxyServiceCaller({self._sac_segmentation_srv_topic: SacSegmentation})

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

                return "continue"

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo("Number of pointclouds in list: {}".format(len(userdata.pointcloud_list)))
                filtered_pointcloud = self._concatenate_pointcloud_srv.call(self._concatenate_pointcloud_srv_topic, userdata.pointcloud_list)
                filtered_pointcloud = self._passthrough_filter_srv.call(self._passthrough_filter_srv_topic, filtered_pointcloud, self._x_min, self._x_max, self._y_min, self._y_max, self._z_min, self._z_max)
                filtered_pointcloud = self._sac_segmentation_srv.call(self._sac_segmentation_srv_topic, filtered_pointcloud)

                userdata.combined_pointcloud = filtered_pointcloud

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

