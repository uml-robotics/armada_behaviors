#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class stageHandlerState(EventState):
        '''
        Put description here.

        -- parameter            type            Description of parameter.

        #> input_key            type            Description of input key.

        #> output_key           type            Description of output key.

        <= outcome                              Description of possible result of execution.

        '''

        def __init__(self):
        # def __init__(self, parameter):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                # super(stageHandlerState, self).__init__(outcomes = ['result_1', 'result_2'],
                #                                        input_keys = ['input_userdata'],
                #                                        output_keys = ['output_userdata'])
                # userdata is accessed by:
                #    userdata.input_userdata,
                #    userdata.output_userdata,
                #    etc.

                super(stageHandlerState, self).__init__(outcomes = ['object_handler', 'perception_handler', 'manipulation_handler', 'finished'],
                                                        input_keys = ['behavior_stage', 'object_stage', 'perception_stage', 'manipulation_stage'],
                                                        output_keys = ['behavior_stage', 'object_stage', 'perception_stage', 'manipulation_stage'])

                # access and store state behavior parameters in self
                # self._parameter = parameter


        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                match userdata.perception_stage:
                    case 0:
                        Logger.loginfo('case 0, object handler')
                        return 'object_handler'
                    case 1:
                        Logger.loginfo('case 1, perception handler')
                        return 'perception_handler'
                    case 2:
                        Logger.loginfo('case 2, manipulation handler')
                        return 'manipulation_handler'
                    case 3:
                        Logger.loginfo('case 3, finished (should check for repetitions)')
                        return 'finished'
                    case _:
                        Logger.loginfo('exception case, something broke')
                        return 'finished'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('This is the main stage handler...' )

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

