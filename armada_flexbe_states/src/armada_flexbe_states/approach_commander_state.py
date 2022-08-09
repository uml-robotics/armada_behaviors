#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

class ApproachCommanderState(EventState):
  '''
  This state will iterate through a list of armada_flexbe_utilities/GraspPoses items and attempt to find a set of approach pose waypoints
  that the robot can complete. Upon approaching the object successfully, the commander will initiate gripper closing to complete
  a grasp. After a successful grasp, the current_list_position userdata will be used as input to execute the retreat motion.

  ># grasp_task_candidates            GraspPoses[]    List of GraspPoses (candidates for full set of grasp task waypoints).
  ># grasp_attempt                    int             Position in grasp_task_candidates list list.
  #> target_pose_list                 Pose[]          The list of target grasp action waypoints
  #> gripper_target_position          int             The target gripper gap size (in meters)
  #> grasp_attempt                    int             Position in grasp_task_candidates list list.

  <= continue                                         Grasping operation is complete.
  <= failed                                           Something went wrong.

  '''

  def __init__(self):
          # See example_state.py for basic explanations.
          super(ApproachCommanderState, self).__init__(outcomes = ['continue', 'failed'],
                                                   input_keys = ['grasp_task_candidates', 'grasp_attempt'],
                                                   output_keys = ['target_pose_list', 'gripper_target_position', 'grasp_attempt'])

  def execute(self, userdata):
          # This method is called periodically while the state is active.
          # Main purpose is to check state conditions and trigger a corresponding outcome.
          # If no outcome is returned, the state will stay active.

          list_size = len(userdata.grasp_task_candidates)

          # empty the list before appending
          userdata.target_pose_list = []

          if userdata.grasp_attempt < list_size:
            userdata.target_pose_list.append(userdata.grasp_task_candidates[userdata.grasp_attempt].pre)
            userdata.target_pose_list.append(userdata.grasp_task_candidates[userdata.grasp_attempt].target)
            userdata.gripper_target_position = 0.8
            return 'continue'
          else:
            return 'failed'


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
