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
from armada_flexbe_states.gripper_control_state import gripperControlState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.snapshot_commander_state import snapshotCommanderState
from armada_flexbe_states.spawn_model_state import spawnObjectState
from flexbe_states.wait_state import WaitState
from sandbox_flexbe_states.step_iterator_state import stepIteratorState
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
		self.add_parameter('gripper_topic', '/r2f85_gripper_controller/gripper_cmd/goal')

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
		_state_machine.userdata.combined_pointcloud = 0
		_state_machine.userdata.gripper_open = 0.0
		_state_machine.userdata.gripper_closed = 0.8

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:63
			OperatableStateMachine.add('SpawnObject',
										spawnObjectState(model_name=self.model_name, object_file_path=self.object_file_path, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})

			# x:894 y:63
			OperatableStateMachine.add('ConcatenatePointCloud',
										concatenatePointCloudState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud'})

			# x:674 y:35
			OperatableStateMachine.add('DeleteObject',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'ConcatenatePointCloud', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low})

			# x:239 y:61
			OperatableStateMachine.add('MoveArm',
										MoveArmActionState(),
										transitions={'finished': 'OpenGripper', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_list': 'initial_pose'})

			# x:406 y:159
			OperatableStateMachine.add('MoveToSnapshotPose',
										MoveArmActionState(),
										transitions={'finished': 'getPointCloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_list': 'target_pose'})

			# x:711 y:375
			OperatableStateMachine.add('OpenGripper',
										gripperControlState(gripper_topic=self.gripper_topic, text='Target gripper value: {}', severity=Logger.REPORT_HINT),
										transitions={'continue': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_val': 'gripper_open'})

			# x:453 y:33
			OperatableStateMachine.add('SnapshotCommander',
										snapshotCommanderState(),
										transitions={'continue': 'DeleteObject', 'take_snapshot': 'MoveToSnapshotPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'take_snapshot': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'target_pose': 'target_pose'})

			# x:459 y:339
			OperatableStateMachine.add('SnapshotStepIterator',
										stepIteratorState(),
										transitions={'continue': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'iterator_in': 'current_snapshot_step', 'iterator_out': 'current_snapshot_step'})

			# x:400 y:250
			OperatableStateMachine.add('getPointCloud',
										getPointCloudState(camera_topic=self.camera_topic),
										transitions={'continue': 'SnapshotStepIterator', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'pointcloud_list': 'pointcloud_list'})

			# x:75 y:192
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'CloseGripper'},
										autonomy={'done': Autonomy.Low})

			# x:57 y:316
			OperatableStateMachine.add('CloseGripper',
										gripperControlState(gripper_topic=self.gripper_topic, text='Target gripper value: {}', severity=Logger.REPORT_HINT),
										transitions={'continue': 'MoveArm', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'target_pose_val': 'gripper_closed'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
