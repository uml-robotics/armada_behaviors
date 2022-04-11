#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class paramToOutputKeyState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- param_to_pass              float            The gripper command topic being used
        -- text  	              string 	       The message to be logged to the terminal Example:  'Counter value:  {}'

        #> target_pose_val                             Target gripper position value (between 0 and 1)

        <= continue 			               Task completed successfully
        <= failed 			               Something went wrong.

        '''

        def __init__(self, param_to_pass, text, severity=Logger.REPORT_HINT):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(paramToOutputKeyState, self).__init__(outcomes = ['continue', 'failed'],
                                                        output_keys = ['target_pose_val'])

                self._param_to_pass = param_to_pass
                self._text = text
                self._severity = severity

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                  closeVal = self._param_to_pass
                  closeVal = closeVal / 100.00
                  userdata.target_pose_val = closeVal
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                pass # Nothing to do in this state.

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                # log the output key while exiting
                Logger.log(self._text.format(userdata.target_pose_val), self._severity)

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                pass # Nothing to do in this state.

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.

