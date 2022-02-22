#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
import random


class randomXYState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- target_time 	float 	Time which needs to have passed since the behavior started.

        #> x            int             random x position.
        #> y            int             random y position.

        <= continue 			generated x,y position.
        <= failed 			something went wrong.

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(randomXYState, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['pose'])

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                pose_x_list = [-0.1, -0.1, 0.1, 0.1, 0.0]
                pose_y_list = [-0.2, -0.2, 0.0, 0.0, -0.1]

                # pose_x = random.choice(pose_x_list)
                pose_x = round(random.uniform(-00.10, 00.10), 2)
                # pose_y = random.choice(pose_y_list)
                pose_y = round(random.uniform(-00.20, 00.00), 2)

                userdata.pose = [pose_x, pose_y, 1.2]

                return 'continue'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('generating random x,y pose from list')

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

