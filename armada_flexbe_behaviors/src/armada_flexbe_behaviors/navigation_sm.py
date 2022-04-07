#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.move_base_state import MoveBaseState as armada_flexbe_states__MoveBaseState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Tue Feb 22 2022
@author: Kevin Yassini
'''
class NavigationSM(Behavior):
	'''
	Given the robot's initial location, navigate to the next stage in pick & place.

Three stages (home, pick, place) can be defined in the userdata: 
Pose2D(x, y, theta) - in map frame.
	'''


	def __init__(self):
		super(NavigationSM, self).__init__()
		self.name = 'Navigation'

		# parameters of this behavior
		self.add_parameter('current_location', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 76 90 
		# Given the robot's initial location (home, pick, place), send the robot to the next stage.|n|nIf current_location is None, the robot will go home.

		# O 191 410 
		# Go again if:|n1. We don't reach the goal|n2. Or we arrived home, so now we need to pick next

		# O 825 115 
		# Reached the goal, update and store the current location for use

		# O 529 523 
		# We arrived at pick or place, move on to manipulation



	def create(self):
		# x:507 y:580, x:446 y:581
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.home = Pose2D(0,0,0)
		_state_machine.userdata.pick = Pose2D(1,1,180)
		_state_machine.userdata.place = Pose2D(2,2,180)
		_state_machine.userdata.next_location = None
		_state_machine.userdata.current_location = self.current_location

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:35 y:164
			OperatableStateMachine.add('Log Current Location',
										LogKeyState(text="Current Location: {}", severity=Logger.REPORT_HINT),
										transitions={'done': 'Decide Next Location'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'current_location'})

			# x:462 y:368
			OperatableStateMachine.add('Check if home',
										CheckConditionState(predicate=self.check_home),
										transitions={'true': 'Decide Next Location', 'false': 'finished'},
										autonomy={'true': Autonomy.Full, 'false': Autonomy.Off},
										remapping={'input_value': 'current_location'})

			# x:231 y:164
			OperatableStateMachine.add('Decide Next Location',
										DecisionState(outcomes=["pick","place","home"], conditions=self.decide_next_location),
										transitions={'pick': 'MovePick', 'place': 'MovePlace', 'home': 'MoveHome'},
										autonomy={'pick': Autonomy.Off, 'place': Autonomy.Off, 'home': Autonomy.Off},
										remapping={'input_value': 'current_location'})

			# x:82 y:366
			OperatableStateMachine.add('Log Failure',
										LogState(text="NAV FAILED, TRYING AGAIN...", severity=Logger.REPORT_ERROR),
										transitions={'done': 'Log Current Location'},
										autonomy={'done': Autonomy.Off})

			# x:914 y:276
			OperatableStateMachine.add('Log Success',
										LogKeyState(text="Nav Success! Robot Reached: {}", severity=Logger.REPORT_HINT),
										transitions={'done': 'Check if home'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'current_location'})

			# x:583 y:101
			OperatableStateMachine.add('MoveHome',
										armada_flexbe_states__MoveBaseState(),
										transitions={'arrived': 'Arrived, Update Current Location', 'failed': 'Log Failure'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'home'})

			# x:584 y:239
			OperatableStateMachine.add('MovePick',
										armada_flexbe_states__MoveBaseState(),
										transitions={'arrived': 'Arrived, Update Current Location', 'failed': 'Log Failure'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pick'})

			# x:583 y:173
			OperatableStateMachine.add('MovePlace',
										armada_flexbe_states__MoveBaseState(),
										transitions={'arrived': 'Arrived, Update Current Location', 'failed': 'Log Failure'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'place'})

			# x:847 y:159
			OperatableStateMachine.add('Arrived, Update Current Location',
										CalculationState(calculation=self.update_current_location),
										transitions={'done': 'Log Success'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'next_location', 'output_value': 'current_location'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def decide_next_location(self, input):
		"""Determine Next location given the current location"""
		if input == "home":
			self.next_location = "pick"
		elif input == "pick":
			self.next_location = "place"
		else:
			self.next_location = "home"

		Logger.loghint("Moving to: %s" % self.next_location)
		return self.next_location

	def update_current_location(self, input):
		return self.next_location

	def check_home(self, input):
		return input == "home"

	# [/MANUAL_FUNC]
