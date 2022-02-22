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
from armada_flexbe_states.random_xy_state import randomXYState
from armada_flexbe_states.spawn_model_state import spawnObjectState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 14 2022
@author: Brian Flynn
'''
class GazeboPickAndPlaceSM(Behavior):
	'''
	Spawn an object, perform grasp/pick and place, delete object, reset.
	'''


	def __init__(self):
		super(GazeboPickAndPlaceSM, self).__init__()
		self.name = 'GazeboPickAndPlace'

		# parameters of this behavior
		self.add_parameter('waiting_time', 1)
		self.add_parameter('model_name', 'coke_can')
		self.add_parameter('object_file_path', '/home/brian/.gazebo/models/coke_can/model.sdf')
		self.add_parameter('robot_namespace', '')
		self.add_parameter('reference_frame', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hello = "Hello World!"
		# x:17 y:538, x:385 y:266
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:47
			OperatableStateMachine.add('initialWait',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'randomXY'},
										autonomy={'done': Autonomy.Off})

			# x:140 y:426
			OperatableStateMachine.add('deleteObject',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'endWait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:162 y:519
			OperatableStateMachine.add('endWait',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:147 y:133
			OperatableStateMachine.add('randomXY',
										randomXYState(),
										transitions={'continue': 'spawnObject', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:147 y:220
			OperatableStateMachine.add('spawnObject',
										spawnObjectState(model_name=self.model_name, object_file_path=self.object_file_path, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'delay', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:168 y:322
			OperatableStateMachine.add('delay',
										WaitState(wait_time=self.waiting_time),
										transitions={'done': 'deleteObject'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
