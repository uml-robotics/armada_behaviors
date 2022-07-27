#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.gripper_command_action_state import GripperCommandActionState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: Brian Flynn
'''
class GripperTestBehaviorSM(Behavior):
	'''
	Testing gripper functionality without arm movement
	'''


	def __init__(self):
		super(GripperTestBehaviorSM, self).__init__()
		self.name = 'GripperTestBehavior'

		# parameters of this behavior
		self.add_parameter('wait_time', 2)
		self.add_parameter('gripper_topic', '/r2f85_gripper_controller/gripper_cmd')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:114 y:377, x:433 y:155
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.gripper_state = 0
		_state_machine.userdata.open = 0
		_state_machine.userdata.close = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:56 y:43
			OperatableStateMachine.add('GripperClose',
										GripperCommandActionState(gripper_topic=self.gripper_topic),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_val': 'close', 'gripper_state': 'gripper_state'})

			# x:57 y:228
			OperatableStateMachine.add('GripperOpen',
										GripperCommandActionState(gripper_topic=self.gripper_topic),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_val': 'open', 'gripper_state': 'gripper_state'})

			# x:82 y:134
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'GripperOpen'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
