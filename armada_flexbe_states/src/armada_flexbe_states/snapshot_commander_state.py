#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose


class snapshotCommanderState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- snapshot_pose_list               string          List of snapshot poses.

        #> current_snapshot_step            int             Position in snapshot list.
        ># target_pose                      string          Target move_to pose.

        <= continue                                         Snapshot operation is complete.
        <= take_snapshot                                    Continue pointcloud collection.
        <= failed                                           Something went wrong.

        '''

        def __init__(self, snapshot_pose_list):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(snapshotCommanderState, self).__init__(outcomes = ['continue', 'take_snapshot', 'failed'],
                                                             input_keys = ['current_snapshot_step'],
                                                             output_keys = ['target_pose'])

                self._snapshot_pose_list = snapshot_pose_list
                self._pose_list_size = len(snapshot_pose_list)

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                if userdata.current_snapshot_step < self._pose_list_size:
                    userdata.target_pose = self.snapshot_pose_list[userdata.current_snapshot_step]
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

