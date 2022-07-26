#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose


class DeleteModelServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- model_name 	string          Name of the model to be deleted from the scene.

        #> x            int             random x position.
        #> y            int             random y position.

        <= continue 			generated x,y position.
        <= failed 			something went wrong.

        '''

        def __init__(self, model_name):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(DeleteModelServiceState, self).__init__(outcomes = ['continue', 'failed'])

                # store object spawn pose info from previous state
                self._model_name = model_name

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                # check/wait for delete model service to spin up
                rospy.wait_for_service('gazebo/delete_model')
                delete_model_srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

                try:
                  delete_model_srv(self._model_name)
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                Logger.loginfo('attempting to spawn object...' )

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

