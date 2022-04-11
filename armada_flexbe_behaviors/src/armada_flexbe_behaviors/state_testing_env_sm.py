#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.cartesian_path_action_state import CartesianPathActionState
from armada_flexbe_states.gripper_control_state import gripperControlState
from armada_flexbe_states.param_to_output_key_state import paramToOutputKeyState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Apr 08 2022
@author: brian flynn
'''
class state_testing_envSM(Behavior):
	'''
	testing state functionality before implementation in the final behavior
	'''


	def __init__(self):
		super(state_testing_envSM, self).__init__()
		self.name = 'state_testing_env'

		# parameters of this behavior
		self.add_parameter('target_gripper_val', 0)
		self.add_parameter('gripper_command_topic', '/r2f85_gripper_controller/gripper_cmd/goal')
		self.add_parameter('wait_time', 1)
		self.add_parameter('log_text', 'log val: {}')
		self.add_parameter('target_named_pose', 'wait')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:725 y:332, x:509 y:334
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pose_val = "wait"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:78 y:77
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'passGripperCloseVal'},
										autonomy={'done': Autonomy.Off})

			# x:705 y:138
			OperatableStateMachine.add('namedMovePlaceholder',
										CartesianPathActionState(topic='move_to_named_pose', pose_name=self.target_named_pose),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:223 y:144
			OperatableStateMachine.add('passGripperCloseVal',
										paramToOutputKeyState(param_to_pass=self.target_gripper_val, text=self.log_text, severity=Logger.REPORT_HINT),
										transitions={'continue': 'gripperControl', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_val': 'target_pose_val'})

			# x:446 y:84
			OperatableStateMachine.add('gripperControl',
										gripperControlState(gripper_topic=self.gripper_command_topic, text=self.log_text, severity=Logger.REPORT_HINT),
										transitions={'continue': 'namedMovePlaceholder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_val': 'target_pose_val'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
