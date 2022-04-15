#!/usr/bin/env python
import rospy
import random

from flexbe_core import EventState, Logger
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


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

        ># pointcloud_list_in                           List of PointCloud2 messages
        #> pointcloud_out                               Filtered & concatenated PointCloud2 message

        <= continue                                     spawned/deleted an object successfully
        <= failed                                       something went wrong

        '''

        def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(concatenatePointCloudState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_list_in'],
                                                       output_keys = ['pointcloud_out'])

                # Set up get_pointcluod service proxy
                rospy.wait_for_service('/get_pointcloud')
                self._concatenate_pointcloud_srv = rospy.ServiceProxy('/concatenate_pointcloud', ConcatenatePointCloud)
                self._passthrough_filter_srv = rospy.ServiceProxy('/passthrough_filter', PointCloudPassthroughFilter)
                self._sac_segmentation_srv = rospy.ServiceProxy('/sac_segmentation', SacSegmentation)

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

                filtered_pointcloud = PointCloud2()

                try:
                    filtered_pointcloud = self._concatenate_pointcloud_srv(userdata.pointcloud_list_in)
                  return 'continue'
                except:
                  return 'failed'

                try:
                    filtered_pointcloud = self._passthrough_filter_srv(filtered_pointcloud, self._x_min, self._x_max, self._y_min, self._y_max, self._z_min, self._z_max)
                  return 'continue'
                except:
                  return 'failed'

                try:
                    filtered_pointcloud = self._sac_segmentation_srv(filtered_pointcloud)
                  return 'continue'
                except:
                  return 'failed'

                userdata.pointcloud_out = filtered_pointcloud

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                pass # Nothing to do in this state.

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

