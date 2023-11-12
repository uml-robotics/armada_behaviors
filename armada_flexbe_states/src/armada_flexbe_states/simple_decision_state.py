#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

class SimpleDecisionState(EventState):
  '''
  This state will assist in controling the flow of the behavior based on the value of a boolean variable or through manual control.

  <= success                                          Some operation succeeded.
  <= failure                                          Some operation failed.

  '''

  def __init__(self):
          # See example_state.py for basic explanations.
          super(SimpleDecisionState, self).__init__(outcomes = ['success', 'failure'],
                                                   input_keys = ['success_bool'])

  def execute(self, userdata):
          # This method is called periodically while the state is active.
          # Main purpose is to check state conditions and trigger a corresponding outcome.
          # If no outcome is returned, the state will stay active.

          if userdata.success_bool:
            return 'success'
          else:
            return 'failure'

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