#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.concatenate_pointcloud_service_state import concatenatePointCloudState
from armada_flexbe_states.delete_model_state import deleteObjectState
from armada_flexbe_states.get_pointcloud_service_state import getPointCloudState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.snapshot_commander_state import snapshotCommanderState
from armada_flexbe_states.spawn_model_state import spawnObjectState
from armada_flexbe_states.step_iterator_state import stepIteratorState
from flexbe_states.log_key_state import LogKeyState
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
		self.add_parameter('camera_topic', '/camera_wrist/depth/points')
		self.add_parameter('concatenated_cloud_topic', '/combined_cloud')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:836 y:656, x:309 y:602
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.initial_pose = ['wait']
		_state_machine.userdata.snapshot_pose_list = ['above','robot_left','robot_right']
		_state_machine.userdata.target_pose = ['']
		_state_machine.userdata.current_snapshot_step = 0
		_state_machine.userdata.pointcloud_list = []
		_state_machine.userdata.pointcloud_out = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:63
			OperatableStateMachine.add('SpawnObject',
										spawnObjectState(model_name=self.model_name, object_file_path=self.object_file_path, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})

			# x:40 y:652
			OperatableStateMachine.add('DeleteObject',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})

			# x:547 y:243
			OperatableStateMachine.add('DisplayIterator',
										LogKeyState(text='iterator is at: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'SnapshotCommander'},
										autonomy={'done': Autonomy.Low},
										remapping={'data': 'current_snapshot_step'})

			# x:239 y:61
			OperatableStateMachine.add('MoveArm',
										MoveArmActionState(),
										transitions={'finished': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_list': 'initial_pose'})

			# x:406 y:159
			OperatableStateMachine.add('MoveToSnapshotPose',
										MoveArmActionState(),
										transitions={'finished': 'getPointCloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_list': 'target_pose'})

			# x:437 y:61
			OperatableStateMachine.add('SnapshotCommander',
										snapshotCommanderState(),
										transitions={'continue': 'ConcatenatePointCloud', 'take_snapshot': 'MoveToSnapshotPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'take_snapshot': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'target_pose': 'target_pose'})

			# x:426 y:324
			OperatableStateMachine.add('SnapshotStepIterator',
										stepIteratorState(),
										transitions={'continue': 'DisplayIterator', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'iterator_in': 'current_snapshot_step', 'iterator_out': 'current_snapshot_step'})

			# x:400 y:250
			OperatableStateMachine.add('getPointCloud',
										getPointCloudState(camera_topic=self.camera_topic),
										transitions={'continue': 'SnapshotStepIterator', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'pointcloud_list_in': 'pointcloud_list', 'pointcloud_list_out': 'pointcloud_list'})

			# x:75 y:192
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'MoveArm'},
										autonomy={'done': Autonomy.Low})

			# x:894 y:63
			OperatableStateMachine.add('ConcatenatePointCloud',
										concatenatePointCloudState(x_min=0, x_max=0, y_min=0, y_max=0, z_min=0, z_max=0),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'pointcloud_list_in': 'pointcloud_list', 'pointcloud_out': 'pointcloud_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
