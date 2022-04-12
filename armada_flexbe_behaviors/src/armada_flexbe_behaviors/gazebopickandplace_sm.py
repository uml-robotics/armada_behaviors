#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.delete_model_state import deleteObjectState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.spawn_model_state import spawnObjectState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 11 2022
@author: Brian Flynn
'''
class GazeboPickAndPlaceSM(Behavior):
	'''
	Perform a pick and place option with a robot arm and a simulated object, spawned in a semi-random location within the usable workspace, for testing and behavior/functionality proofing
	'''


	def __init__(self):
		super(GazeboPickAndPlaceSM, self).__init__()
		self.name = 'GazeboPickAndPlace'

		# parameters of this behavior
		self.add_parameter('model_name', 'coke_can')
		self.add_parameter('object_file_path', '/home/.gazebo/models/coke_can/model.sdf')
		self.add_parameter('robot_namespace', '')
		self.add_parameter('reference_frame', 'world')
		self.add_parameter('wait_time', 2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:531 y:317, x:208 y:313
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pose_list = ['retract','wait','above','robot_left','robot_right']

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:63
			OperatableStateMachine.add('SpawnObject',
										spawnObjectState(model_name=self.model_name, object_file_path=self.object_file_path, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})

			# x:443 y:57
			OperatableStateMachine.add('MoveArm',
										MoveArmActionState(),
										transitions={'finished': 'DeleteObject', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_list': 'target_pose_list'})

			# x:266 y:62
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'MoveArm'},
										autonomy={'done': Autonomy.Low})

			# x:713 y:57
			OperatableStateMachine.add('DeleteObject',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
