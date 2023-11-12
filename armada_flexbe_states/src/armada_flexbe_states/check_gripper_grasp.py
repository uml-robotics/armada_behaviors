#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input


class CheckGripperGraspState(EventState):
        '''
        Check if the gripper has grasped/is holding an object. This will only work for grippers that have some functionality to detect grasp success

        <= success                                       gripper is probably holding object (fingers stopped before reaching setpoint)
        <= failure                                       gripper did not grasp object (fingers closed all the way)

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(CheckGripperGraspState, self).__init__(outcomes = ['success', 'failure'])

                self._gripper_status_topic = rospy.get_param("/end_effector/gripper_status_topic")
                self._subscriber = ProxySubscriberCached({self._gripper_status_topic: Robotiq2FGripper_robot_input})

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                if self._subscriber.has_msg(self._gripper_status_topic):
                    try:
                        gripper_status = self._subscriber.get_last_msg(self._gripper_status_topic)
                        if gripper_status.gOBJ == 2:
                               return 'success'
                        else:
                               return 'failure'
                    except:
                        Logger.loginfo('topic had no message...')
                        return 'failure'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('checking of gripper has grasped object...' )

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