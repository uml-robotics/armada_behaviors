#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

class snapshotCommanderState(EventState):
        '''
        This state will iterate through a list of snapshot poses that the robot needs to move to
        and initiate either movement or capture of pointcloud data depending on the current step,
        this state should be used in a loop to make decisions.

        #> snapshot_pose_list               string          List of pre-defined (SRDF) poses for camera snapshot(s).
        #> current_snapshot_step            int             Position in snapshot list.
        ># target_pose                      string          Target move_to pose.

        <= continue                                         Snapshot operation is complete.
        <= take_snapshot                                    Continue pointcloud collection.
        <= failed                                           Something went wrong.

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(snapshotCommanderState, self).__init__(outcomes = ['continue', 'take_snapshot', 'failed'],
                                                             input_keys = ['snapshot_pose_list', 'current_snapshot_step'],
                                                             output_keys = ['target_pose'])

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                pose_list_size = len(userdata.snapshot_pose_list)
                if userdata.current_snapshot_step < pose_list_size:
                    next_pose = [userdata.snapshot_pose_list[userdata.current_snapshot_step]]
                    userdata.target_pose = next_pose
                    return 'take_snapshot'
                else:
                    return 'continue'

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

