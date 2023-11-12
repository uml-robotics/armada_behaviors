#!/usr/bin/env python
import rospy

from obswebsocket import obsws, requests

from flexbe_core import EventState, Logger

class OBSStartRecordingState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        <= finished                                     successfully started recording
        <= error                                        failed to start recording

        '''

        def __init__(self):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(OBSStartRecordingState, self).__init__(outcomes = ['finished', 'error'])

                self._HOST = "localhost"
                self._PORT = 4455
                self._PASSWORD = "secret"

                self._obs_recording_scene = rospy.get_param("/data_logging/obs_scene")

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                ws = obsws(self._HOST, self._PORT, self._PASSWORD)
                ws.connect()

                ws.call(requests.SetCurrentProgramScene(sceneName=self._obs_recording_scene))

                try: 
                    ws.call(requests.StartRecord())
                    ws.disconnect()
                    return 'finished'
                except: 
                    ws.disconnect()
                    return 'error'

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