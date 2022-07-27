
#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommand

class GripperCommandActionState(EventState):
        '''
        A state to control the position of a gripper using the GripperCommand action server

        -- gripper_topic              string           The gripper command topic being used
        -- text  	              string 	       The message to be logged to the terminal Example:  'Counter value:  {}'

        ># target_pose_val                             Target gripper position value (between 0 and 1)
        #> gripper_state                                 State of gripper after operation (open or closed)

        <= continue 			               Task completed successfully
        <= failed 			               Something went wrong.

        '''

        def __init__(self, gripper_topic):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(GripperCommandActionState, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['target_pose_val'],
                                                        output_keys = ['gripper_state'])

                self._topic = gripper_topic
                self._client = ProxyActionClient({self._topic: GripperCommandAction})

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
                        reached_goal = result.reached_goal
                        stalled = result.stalled

                        # Based on the result, decide which outcome to trigger.
                        if reached_goal == 1:
                                userdata.gripper_state = userdata.target_pose_val
                                return 'continue'
                        elif stalled == 1:
                                userdata.gripper_state = userdata.target_pose_val
                                return 'continue'
                        else:
                                return 'failed'

                # If the action has not yet finished, no outcome will be returned and the state stays active.


        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                goal = GripperCommandGoal()
                goal.command.position = userdata.target_pose_val
                goal.command.max_effort = 1.0

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

