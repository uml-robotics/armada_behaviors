#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from armada_flexbe_utilities.msg import NamedPoseMoveAction, NamedPoseMoveGoal
from armada_flexbe_utilities.msg import CartesianMoveAction, CartesianMoveGoal

from geometry_msgs.msg import Pose

class MoveArmActionState(EventState):
        '''

        ># target_pose_list                 List of poses, this can be names (pre-defined, named poses) or waypoints (

        <= finished                         Arm successfully moved to named position
        <= failed                           Motion planning/execution failed

        '''

        def __init__(self):
                # See example_state.py for basic explanations.
                super(MoveArmActionState, self).__init__(outcomes = ['finished', 'failed'],
                                                               input_keys = ['target_pose_list'])

                # Store the action server topics for convenience
                self._cartesian_move_action_topic = 'execute_cartesian_plan'
                self._named_pose_move_action_topic = 'move_to_named_pose'
                self._topic = ''
                self._client = ProxyActionClient({self._cartesian_move_action_topic: CartesianMoveAction}) # pass required clients as dict (topic: type)
                self._client = ProxyActionClient({self._named_pose_move_action_topic: NamedPoseMoveAction}) # pass required clients as dict (topic: type)

                # It may happen that the action client fails to send the action goal.
                self._error = False


        def execute(self, userdata):
                # While this state is active, check if the action has been finished and evaluate the result.

                # Check if the client failed to send the goal.
                if self._error:
                        return 'failed'

                # Check if the action has been finished
                if self._client.has_result(self._topic):
                        result = self._client.get_result(self._topic)
                        execution_success = result.execution_success

                        # Based on the result, decide which outcome to trigger.
                        if execution_success == 1:
                                return 'finished'
                        else:
                                return 'failed'

                # If the action has not yet finished, no outcome will be returned and the state stays active.


        def on_enter(self, userdata):
                # When entering this state, we send the action goal once to let the robot start its work.

                # As documented above, we get the specification of which dishwasher to use as input key.
                # This enables a previous state to make this decision during runtime and provide the ID as its own output key.

                # Create the goal.
                firstval = userdata.target_pose_list[0]
                if isinstance(firstval, str):
                  goal = NamedPoseMoveGoal()
                  goal.pose_names = userdata.target_pose_list
                  self._topic = self._named_pose_move_action_topic
                elif isinstance(firstval, Pose):
                  goal = CartesianMoveGoal()
                  goal.grasp_waypoints = userdata.target_pose_list
                  self._topic = self._cartesian_move_action_topic

                # Send the goal.
                self._error = False # make sure to reset the error state since a previous state execution might have failed
                try:
                        self._client.send_goal(self._topic, goal)
                except Exception as e:
                        # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
                        # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
                        Logger.logwarn('Failed to send the command:\n%s' % str(e))
                        self._error = True


        def on_exit(self, userdata):
                # Make sure that the action is not running when leaving this state.
                # A situation where the action would still be active is for example when the operator manually triggers an outcome.

                if not self._client.has_result(self._cartesian_move_action_topic):
                        self._client.cancel(self._cartesian_move_action_topic)
                        Logger.loginfo('Cancelled active action goal.')
                if not self._client.has_result(self._named_pose_move_action_topic):
                        self._client.cancel(self._named_pose_move_action_topic)
                        Logger.loginfo('Cancelled active action goal.')
