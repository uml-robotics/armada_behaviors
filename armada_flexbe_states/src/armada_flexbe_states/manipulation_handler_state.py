#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class manipulationHandlerState(EventState):
        '''
        Put description here.

        #> behavior_stage           int            Description of input key.
        #> object_stage             int            Description of input key.
        #> perception_stage         int            Description of input key.
        #> manipulation_stage       int            Description of input key.

        #> behavior_stage           int            Description of output key.
        #> object_stage             int            Description of output key.
        #> perception_stage         int            Description of output key.
        #> manipulation_stage       int            Description of output key.

        <= gen_waypoints                           Description of possible result of execution.
        <= move_arm                                Description of possible result of execution.
        <= gripper_control                         Description of possible result of execution.
        <= finished                                Description of possible result of execution.
        <= failed                                  Description of possible result of execution.

        '''

        def __init__(self):
        # def __init__(self, parameter):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                # super(stageHandlerState, self).__init__(outcomes = ['result_1', 'result_2'],
                #                                        input_keys = ['input_userdata'],
                #                                        output_keys = ['output_userdata'])

                super(manipulationHandlerState, self).__init__(outcomes = ['gen_waypoints', 'move_arm', 'gripper_control', 'finished', 'failed'],
                                                               input_keys = ['behavior_stage', 'object_stage', 'perception_stage', 'manipulation_stage'],
                                                               output_keys = ['behavior_stage', 'object_stage', 'perception_stage', 'manipulation_stage', 'pose_waypoints'])

                # store object spawn pose info from previous state
                # self._parameter = parameter

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                match self._manipulation_stage:
                    case 0:
                        Logger.loginfo('case 0, generate grasp waypoints')
                        return 'gen_waypoints'
                    case 1:
                        Logger.loginfo('case 1, move arm to grasp position')
                        return 'move_arm'
                    case 2:
                        Logger.loginfo('case 2, close gripper')
                        return 'gripper_control'
                    case 3:
                        Logger.loginfo('case 3, move to post-grasp position')
                        return 'move_arm'
                    case 4:
                        Logger.loginfo('case 4, move to dropoff position')
                        return 'move_arm'
                    case 5:
                        Logger.loginfo('case 5, open gripper')
                        return 'gripper_control'
                    case 6:
                        Logger.loginfo('case 6, return to neutral pose')
                        return 'move_arm'
                    case 7:
                        Logger.loginfo('case 7, finished')

                        return 'finished'
                    case _:
                        Logger.loginfo('exception case, something broke')
                        return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('This is the manipulation stage handler...' )

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

