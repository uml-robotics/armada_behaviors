#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

class readWriteUserdataState(EventState):
        '''
        Example for a state which will read some (int) userdata, do something to it, and then output it as some new (int) userdata
        For testing and understanding FlexBe

        -- text  	              string 	       The message to be logged to the terminal Example:  'Counter value:  {}'

        ># input_userdata             int              Some nominal int value used for testing and logging purposes
        #> output_userdata            int              Some nominal int value used for testing and logging purposes

        <= continue 			               Task completed successfully
        <= failed 			               Something went wrong

        '''

        def __init__(self, text, severity=Logger.REPORT_HINT):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(readWriteUserdataState, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['input_userdata'],
                                                        output_keys = ['output_userdata'])

                self._text = text
                self._severity = severity

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                try:
                  userdata.output_userdata = userdata.input_userdata * 5
                  Logger.log(self._text.format(userdata.output_userdata), self._severity)
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

