#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

class EuclideanClusterExtractionServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- param                        string          Param description

        ># input_key                                    Input Key info
        #> output_key                                   Output Key info

        <= continue                                     Continue outcome description
        <= failed                                       Failure outcome description

        '''

        def __init__(self, example_param):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(EuclideanClusterExtractionServiceState, self).__init__(outcomes = ['continue', 'failed'],
                                                       input_keys = ['input_key'],
                                                       output_keys = ['output_key'])

                self._example_param = example_param

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                pass # Add functionality here if necessary

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                pass # Add functionality here if necessary

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Add functionality here if necessary

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                pass # Add functionality here if necessary

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Add functionality here if necessary

