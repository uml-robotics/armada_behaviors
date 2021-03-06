#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.calculate_grasp_waypoints_state import calculateGraspWaypointsServiceState
from armada_flexbe_states.concatenate_pointcloud_service_state import concatenatePointCloudState
from armada_flexbe_states.delete_model_state import deleteObjectState
from armada_flexbe_states.get_grasp_candidates_service_state import getGraspCandidateState
from armada_flexbe_states.get_pointcloud_service_state import getPointCloudState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.pointcloud_passthrough_filter_service_state import pointCloudPassthroughFilterState
from armada_flexbe_states.publish_pointcloud_state import publishPointCloudState
from armada_flexbe_states.sac_segmentation_service_state import pointCloudSacSegmentationState
from armada_flexbe_states.snapshot_commander_state import snapshotCommanderState
from armada_flexbe_states.spawn_model_state import spawnObjectState
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
		self.add_parameter('grasp_candidates_topic', '/detect_grasps/clustered_grasps')
		self.add_parameter('grasp_offset', 0.165)
		self.add_parameter('pregrasp_dist', 0.10)
		self.add_parameter('postgrasp_dist', 0.10)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:879 y:593, x:246 y:464
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.initial_pose = ['wait']
		_state_machine.userdata.snapshot_pose_list = ['above','robot_left','robot_right']
		_state_machine.userdata.target_pose = ['']
		_state_machine.userdata.current_snapshot_step = 0
		_state_machine.userdata.pointcloud_list = []
		_state_machine.userdata.combined_pointcloud = 0
		_state_machine.userdata.gripper_open = 0.0
		_state_machine.userdata.gripper_closed = 0.8
		_state_machine.userdata.grasp_candidates = []
		_state_machine.userdata.grasp_waypoints_list = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:31
			OperatableStateMachine.add('DeleteObjectStart',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'SpawnObject', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:801 y:56
			OperatableStateMachine.add('ConcatenatePointCloud',
										concatenatePointCloudState(),
										transitions={'continue': 'PointCloudPassthroughFilter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud'})

			# x:825 y:495
			OperatableStateMachine.add('DeleteObjectEnd',
										deleteObjectState(model_name=self.model_name),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:813 y:412
			OperatableStateMachine.add('GetGraspCandidates',
										getGraspCandidateState(combined_cloud_topic=self.concatenated_cloud_topic, grasp_candidates_topic=self.grasp_candidates_topic),
										transitions={'continue': 'CalculateGraspWaypoints', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'combined_pointcloud': 'combined_pointcloud', 'grasp_candidates': 'grasp_candidates'})

			# x:239 y:61
			OperatableStateMachine.add('MoveArm',
										MoveArmActionState(),
										transitions={'finished': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'initial_pose'})

			# x:615 y:59
			OperatableStateMachine.add('MoveArm2',
										MoveArmActionState(),
										transitions={'finished': 'ConcatenatePointCloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'initial_pose'})

			# x:407 y:162
			OperatableStateMachine.add('MoveToSnapshotPose',
										MoveArmActionState(),
										transitions={'finished': 'getPointCloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'target_pose'})

			# x:784 y:153
			OperatableStateMachine.add('PointCloudPassthroughFilter',
										pointCloudPassthroughFilterState(x_min=-1.125, x_max=-0.225, y_min=-0.6, y_max=0.6, z_min=-0.1, z_max=0.15),
										transitions={'continue': 'PointCloudSacSegmentation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:784 y:246
			OperatableStateMachine.add('PointCloudSacSegmentation',
										pointCloudSacSegmentationState(),
										transitions={'continue': 'PublishPointCloud', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:814 y:327
			OperatableStateMachine.add('PublishPointCloud',
										publishPointCloudState(topic=self.concatenated_cloud_topic),
										transitions={'continue': 'GetGraspCandidates', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'combined_pointcloud'})

			# x:422 y:62
			OperatableStateMachine.add('SnapshotCommander',
										snapshotCommanderState(),
										transitions={'continue': 'MoveArm2', 'take_snapshot': 'MoveToSnapshotPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'take_snapshot': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'target_pose': 'target_pose'})

			# x:633 y:317
			OperatableStateMachine.add('SnapshotStepIterator',
										stepIteratorState(),
										transitions={'continue': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'iterator_in': 'current_snapshot_step', 'iterator_out': 'current_snapshot_step'})

			# x:60 y:100
			OperatableStateMachine.add('SpawnObject',
										spawnObjectState(model_name=self.model_name, object_file_path=self.object_file_path, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'MoveArm', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:408 y:256
			OperatableStateMachine.add('getPointCloud',
										getPointCloudState(camera_topic=self.camera_topic),
										transitions={'continue': 'SnapshotStepIterator', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list'})

			# x:1009 y:83
			OperatableStateMachine.add('CalculateGraspWaypoints',
										calculateGraspWaypointsServiceState(grasp_offset=self.grasp_offset, pregrasp_dist=self.pregrasp_dist, postgrasp_dist=self.postgrasp_dist),
										transitions={'continue': 'DeleteObjectEnd', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_candidates': 'grasp_candidates', 'grasp_waypoints_list': 'grasp_waypoints_list'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
