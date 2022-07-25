#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

class GraspCommanderState(EventState):
  '''
  This state will iterate through a list of armada_flexbe_utilities/GraspPoses items and attempt to find a set of approach pose waypoints
  that the robot can complete. Upon approaching the object successfully, the commander will initiate gripper closing to complete
  a grasp. After a successful grasp, the current_list_position userdata will be used as input to execute the retreat motion.

  #> grasp_waypoints_list             string          List of GraspPoses.
  #> current_list_position            int             Position in snapshot list.
  ># target_pose_list                 Pose            List of grasp waypoint Poses.

  <= continue                                         Grasping operation is complete.
  <= attempt_grasp                                    Send list of waypoint Poses to move_arm action state.
  <= close_gripper                                    Go to close gripper service state.
  <= failed                                           Something went wrong.

  '''

        def __init__(self):
                # See example_state.py for basic explanations.
                super(GraspCommanderState, self).__init__(outcomes = ['continue', 'attempt_grasp', 'close_gripper', 'failed'],
                                                         input_keys = ['grasp_waypoints_list', 'current_list_position'],
                                                         output_keys = ['target_pose_list'])

         def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                pose_list_size = len(userdata.snapshot_pose_list)
                if userdata.current_snapshot_step < pose_list_size:
                    next_pose = [userdata.snapshot_pose_list[userdata.current_snapshot_step]]
                    userdata.target_pose = next_pose
                    return 'attempt_grasp'
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
