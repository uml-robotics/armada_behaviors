#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class perceptionHandlerState(EventState):
        '''
        Put description here.

        -- parameter            type            Description of parameter.

        #> input_key            type            Description of input key.

        #> output_key           type            Description of output key.

        <= outcome                              Description of possible result of execution.

        '''

        def __init__(self, snapshot_pose_list):
        # def __init__(self, parameter):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                # super(stageHandlerState, self).__init__(outcomes = ['result_1', 'result_2'],
                #                                        input_keys = ['input_userdata'],
                #                                        output_keys = ['output_userdata'])
                # userdata is accessed by:
                #    userdata.input_userdata,
                #    userdata.output_userdata,
                #    etc.

                super(perceptionHandlerState, self).__init__(outcomes = ['move_arm', 'process_image', 'finished', 'failed'],
                                                             input_keys = ['behavior_stage', 'perception_stage'],
                                                             output_keys = [''])

                # access and store state behavior parameters in self
                # self._parameter = parameter

                self._snapshot_pose_list = snapshot_pose_list

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                match userdata.perception_stage:
                    case 0:
                        Logger.loginfo('case 0, move arm to snapshot pose')
                        return 'move_arm'
                    case 1:
                        Logger.loginfo('case 1, generate grasp waypoints')
                        if self._current_snapshot_pose_num > self._num_snapshot_poses:
                            # all poses have been reached
                            self._perception_stage += 1
                        else:
                            # go back and move arm again, have not reached all poses
                            self._perception_stage -= 1
                        return 'process_image'
                    case _:
                        Logger.loginfo('exception case, something broke')
                        return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('This is the perception stage handler...' )

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

