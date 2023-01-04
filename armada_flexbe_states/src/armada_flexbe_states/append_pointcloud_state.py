#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class AppendPointCloudState(EventState):
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
                super(AppendPointCloudState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['pointcloud_list', 'pointcloud_in'],
                                                       output_keys = ['pointcloud_list'])

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                    userdata.pointcloud_list.append(userdata.pointcloud_in)
                    return "continue"
                except:
                    return "failed"

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('appending pointcloud to list...' )

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

