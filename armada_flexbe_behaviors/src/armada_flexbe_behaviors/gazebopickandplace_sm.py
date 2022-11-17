#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.approach_commander_state import ApproachCommanderState
from armada_flexbe_states.delete_model_service_state import DeleteModelServiceState
from armada_flexbe_states.get_pointcloud_service_state import GetPointCloudServiceState
from armada_flexbe_states.gpd_grasp_candidates_service_state import GPDGraspCandidatesServiceState
from armada_flexbe_states.gpd_grasp_waypoints_service_state import GPDGraspWaypointsServiceState
from armada_flexbe_states.gripper_command_action_state import GripperCommandActionState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.pcl_concatenate_pointcloud_service_state import PCLConcatenatePointCloudServiceState
from armada_flexbe_states.pcl_euclidean_cluster_extraction_service_state import PCLEuclideanClusterExtractionServiceState
from armada_flexbe_states.pcl_passthrough_filter_service_state import PCLPassthroughFilterServiceState
from armada_flexbe_states.pcl_radius_outlier_removal_service_state import PCLPlaneSegmentationServiceState
from armada_flexbe_states.pointcloud_publisher_state import PointCloudPublisherState
from armada_flexbe_states.retreat_commander_state import RetreatCommanderState
from armada_flexbe_states.snapshot_commander_state import SnapshotCommanderState
from armada_flexbe_states.spawn_model_service_state import SpawnModelServiceState
from flexbe_practice_states.step_iterator_state import stepIteratorState
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
	Perform a pick and place option with a simulated robot arm and a simulated object, spawned in a semi-random location within the usable workspace. For testing and behavior/functionality proofing.
	'''


	def __init__(self):
		super(GazeboPickAndPlaceSM, self).__init__()
		self.name = 'GazeboPickAndPlace'

		# parameters of this behavior
		self.add_parameter('model_name', 'coke_can')
		self.add_parameter('robot_namespace', '')
		self.add_parameter('wait_time', 2)
		self.add_parameter('camera_topic', '/camera_wrist/depth/points')
		self.add_parameter('concatenated_cloud_topic', '/combined_cloud')
		self.add_parameter('gripper_topic', '/r2f85_gripper_controller/gripper_cmd')
		self.add_parameter('grasp_candidates_topic', '/detect_grasps/clustered_grasps')
		self.add_parameter('reference_frame', 'world')
		self.add_parameter('second_model_name', 'coke_can_2')
		self.add_parameter('third_model_name', 'coke_can_3')
		self.add_parameter('Cluster_cloud', 'cluster_cloud')
		self.add_parameter('object_file_path_coke', '/home/csrobot/.gazebo/models/coke_can/model.sdf')
		self.add_parameter('object_file_path_cube', '/home/csrobot/.gazebo/models/coke_can/model.sdf')
		self.add_parameter('object_file_path', '/home/csrobot/.gazebo/models/coke_can/model.sdf')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:360 y:665, x:367 y:351
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.wait_pose = ['wait']
		_state_machine.userdata.snapshot_pose_list = ['above','robot_left','robot_right']
		_state_machine.userdata.target_pose = ['']
		_state_machine.userdata.current_snapshot_step = 0
		_state_machine.userdata.pointcloud_list = []
		_state_machine.userdata.combined_pointcloud = 0
		_state_machine.userdata.grasp_candidates = []
		_state_machine.userdata.grasp_waypoints_list = []
		_state_machine.userdata.grasp_attempt = 0
		_state_machine.userdata.grasp_state = 'approach'
		_state_machine.userdata.gripper_state = 'open'
		_state_machine.userdata.gripper_target_position = 0.0
		_state_machine.userdata.gripper_initial_state = 0.0
		_state_machine.userdata.gripper_actual_position = 0.0
		_state_machine.userdata.dropoff_pose = ['dropoff']

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:117 y:485, x:341 y:247
		_sm_solvegraspwaypointscontainer_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['combined_pointcloud', 'grasp_candidates', 'combined_pointcloud_cluster'], output_keys=['grasp_waypoints_list'])

		with _sm_solvegraspwaypointscontainer_0:
			# x:30 y:40
			OperatableStateMachine.add('PublishPointCloud',
										PointCloudPublisherState(topic=self.concatenated_cloud_topic),
										transitions={'continue': 'PublishCluster', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'combined_pointcloud'})

			# x:15 y:380
			OperatableStateMachine.add('GPDGraspWaypoints',
										GPDGraspWaypointsServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_candidates': 'grasp_candidates', 'grasp_waypoints_list': 'grasp_waypoints_list'})

			# x:386 y:134
			OperatableStateMachine.add('PublishCluster',
										PointCloudPublisherState(topic=self.Cluster_cloud),
										transitions={'continue': 'GPDGraspCandidates', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'combined_pointcloud_cluster'})

			# x:401 y:44
			OperatableStateMachine.add('WaitForNodeRespawn',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'PublishPointCloud'},
										autonomy={'done': Autonomy.Off})

			# x:28 y:275
			OperatableStateMachine.add('GPDGraspCandidates',
										GPDGraspCandidatesServiceState(combined_cloud_topic=self.concatenated_cloud_topic),
										transitions={'continue': 'GPDGraspWaypoints', 'failed': 'WaitForNodeRespawn'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'combined_pointcloud': 'combined_pointcloud', 'grasp_candidates': 'grasp_candidates'})


		# x:385 y:43, x:432 y:199
		_sm_snapshotcontainer_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['snapshot_pose_list', 'current_snapshot_step', 'pointcloud_list', 'combined_pointcloud'], output_keys=['snapshot_pose_list', 'current_snapshot_step', 'pointcloud_list'])

		with _sm_snapshotcontainer_1:
			# x:30 y:40
			OperatableStateMachine.add('SnapshotCommander',
										SnapshotCommanderState(),
										transitions={'continue': 'finished', 'take_snapshot': 'MoveToSnapshotPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'take_snapshot': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'target_pose': 'target_pose'})

			# x:158 y:192
			OperatableStateMachine.add('MoveToSnapshotPose',
										MoveArmActionState(),
										transitions={'finished': 'GetPointCloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'target_pose'})

			# x:23 y:346
			OperatableStateMachine.add('GetPointCloud',
										GetPointCloudServiceState(camera_topic=self.camera_topic),
										transitions={'continue': 'SnapshotCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list'})


		# x:147 y:320, x:349 y:148
		_sm_retreatcontainer_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grasp_waypoints_list', 'grasp_attempt'], output_keys=['target_pose_list', 'gripper_target_position'])

		with _sm_retreatcontainer_2:
			# x:77 y:50
			OperatableStateMachine.add('RetreatCommander',
										RetreatCommanderState(),
										transitions={'continue': 'MoveArmRetreat', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_task_candidates': 'grasp_waypoints_list', 'grasp_attempt': 'grasp_attempt', 'target_pose_list': 'target_pose_list', 'gripper_target_position': 'gripper_target_position'})

			# x:84 y:182
			OperatableStateMachine.add('MoveArmRetreat',
										MoveArmActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'target_pose_list'})


		# x:401 y:366, x:404 y:181
		_sm_pclfiltercontainer_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pointcloud_list', 'combined_pointcloud'], output_keys=['combined_pointcloud', 'pointcloud_list', 'combined_pointcloud_cluster'])

		with _sm_pclfiltercontainer_3:
			# x:30 y:40
			OperatableStateMachine.add('ConcatenatePointCloud',
										PCLConcatenatePointCloudServiceState(),
										transitions={'continue': 'PCLPassthroughFilter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list_in': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud', 'pointcloud_list_out': 'pointcloud_list'})

			# x:43 y:146
			OperatableStateMachine.add('PCLPassthroughFilter',
										PCLPassthroughFilterServiceState(),
										transitions={'continue': 'PointCloudPlanarSegmentation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:39 y:250
			OperatableStateMachine.add('PointCloudPlanarSegmentation',
										PCLPlaneSegmentationServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:27 y:357
			OperatableStateMachine.add('cluster_extraction_state',
										PCLEuclideanClusterExtractionServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_list_out': 'combined_pointcloud_cluster'})


		# x:314 y:377, x:318 y:116
		_sm_initobjectcontainer_4 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_initobjectcontainer_4:
			# x:32 y:63
			OperatableStateMachine.add('InitDeleteObject',
										DeleteModelServiceState(model_name=self.model_name),
										transitions={'continue': 'InitDeleteObject2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:33 y:125
			OperatableStateMachine.add('InitDeleteObject2',
										DeleteModelServiceState(model_name=self.second_model_name),
										transitions={'continue': 'initDeleteObject3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:32 y:250
			OperatableStateMachine.add('InitSpawnObject',
										SpawnModelServiceState(model_name=self.model_name, object_file_path=self.object_file_path_coke, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'SpawnModel2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:33 y:312
			OperatableStateMachine.add('SpawnModel2',
										SpawnModelServiceState(model_name=self.second_model_name, object_file_path=self.object_file_path_cube, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'third_item', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:33 y:187
			OperatableStateMachine.add('initDeleteObject3',
										DeleteModelServiceState(model_name=self.third_model_name),
										transitions={'continue': 'InitSpawnObject', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:34 y:371
			OperatableStateMachine.add('third_item',
										SpawnModelServiceState(model_name=self.third_model_name, object_file_path=self.object_file_path_cube, robot_namespace=self.robot_namespace, reference_frame=self.reference_frame),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:101 y:284, x:234 y:166
		_sm_deleteobjectcontainer_5 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_deleteobjectcontainer_5:
			# x:41 y:67
			OperatableStateMachine.add('WaitToOpenGripper',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'DeleteModel'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:152
			OperatableStateMachine.add('DeleteModel',
										DeleteModelServiceState(model_name=self.model_name),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:203 y:421, x:475 y:110
		_sm_approachcontainer_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grasp_waypoints_list', 'gripper_target_position', 'gripper_actual_position', 'gripper_initial_state', 'grasp_attempt', 'gripper_state'], output_keys=['grasp_attempt', 'gripper_actual_position', 'gripper_state'])

		with _sm_approachcontainer_6:
			# x:30 y:40
			OperatableStateMachine.add('ApproachCommander',
										ApproachCommanderState(),
										transitions={'continue': 'MoveArmGrasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_task_candidates': 'grasp_waypoints_list', 'grasp_attempt': 'grasp_attempt', 'target_pose_list': 'target_pose_list', 'gripper_target_position': 'gripper_target_position'})

			# x:212 y:116
			OperatableStateMachine.add('GraspStepIterator',
										stepIteratorState(),
										transitions={'continue': 'ApproachCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'iterator_in': 'grasp_attempt', 'iterator_out': 'grasp_attempt'})

			# x:140 y:274
			OperatableStateMachine.add('GripperClose',
										GripperCommandActionState(gripper_topic=self.gripper_topic),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'gripper_target_position': 'gripper_target_position', 'gripper_initial_state': 'gripper_initial_state', 'gripper_actual_position': 'gripper_actual_position', 'gripper_state': 'gripper_state'})

			# x:30 y:176
			OperatableStateMachine.add('MoveArmGrasp',
										MoveArmActionState(),
										transitions={'finished': 'GripperClose', 'failed': 'GraspStepIterator'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'target_pose_list'})



		with _state_machine:
			# x:59 y:56
			OperatableStateMachine.add('InitObjectContainer',
										_sm_initobjectcontainer_4,
										transitions={'finished': 'GripperCommandInit', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:48 y:617
			OperatableStateMachine.add('DeleteObjectContainer',
										_sm_deleteobjectcontainer_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:539 y:61
			OperatableStateMachine.add('GripperCommandInit',
										GripperCommandActionState(gripper_topic=self.gripper_topic),
										transitions={'continue': 'MoveArmPreSnapshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'gripper_target_position': 'gripper_target_position', 'gripper_initial_state': 'gripper_initial_state', 'gripper_actual_position': 'gripper_actual_position', 'gripper_state': 'gripper_state'})

			# x:551 y:536
			OperatableStateMachine.add('GripperCommandOpen',
										GripperCommandActionState(gripper_topic=self.gripper_topic),
										transitions={'continue': 'MoveArmPreSnapshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'gripper_target_position': 'gripper_target_position', 'gripper_initial_state': 'gripper_initial_state', 'gripper_actual_position': 'gripper_actual_position', 'gripper_state': 'gripper_state'})

			# x:554 y:406
			OperatableStateMachine.add('MoveArmPostGrasp',
										MoveArmActionState(),
										transitions={'finished': 'GripperCommandOpen', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'dropoff_pose'})

			# x:548 y:240
			OperatableStateMachine.add('MoveArmPostSnapshot',
										MoveArmActionState(),
										transitions={'finished': 'PCLFilterContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'wait_pose'})

			# x:551 y:161
			OperatableStateMachine.add('MoveArmPreSnapshot',
										MoveArmActionState(),
										transitions={'finished': 'SnapshotContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'wait_pose'})

			# x:55 y:233
			OperatableStateMachine.add('PCLFilterContainer',
										_sm_pclfiltercontainer_3,
										transitions={'finished': 'SolveGraspWaypointsContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud', 'combined_pointcloud_cluster': 'combined_pointcloud_cluster'})

			# x:61 y:533
			OperatableStateMachine.add('RetreatContainer',
										_sm_retreatcontainer_2,
										transitions={'finished': 'MoveArmPostGrasp', 'failed': 'GripperCommandOpen'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grasp_waypoints_list': 'grasp_waypoints_list', 'grasp_attempt': 'grasp_attempt', 'target_pose_list': 'target_pose_list', 'gripper_target_position': 'gripper_target_position'})

			# x:57 y:155
			OperatableStateMachine.add('SnapshotContainer',
										_sm_snapshotcontainer_1,
										transitions={'finished': 'MoveArmPostSnapshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud'})

			# x:27 y:340
			OperatableStateMachine.add('SolveGraspWaypointsContainer',
										_sm_solvegraspwaypointscontainer_0,
										transitions={'finished': 'ApproachContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'combined_pointcloud': 'combined_pointcloud', 'grasp_candidates': 'grasp_candidates', 'combined_pointcloud_cluster': 'combined_pointcloud_cluster', 'grasp_waypoints_list': 'grasp_waypoints_list'})

			# x:58 y:437
			OperatableStateMachine.add('ApproachContainer',
										_sm_approachcontainer_6,
										transitions={'finished': 'RetreatContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grasp_waypoints_list': 'grasp_waypoints_list', 'gripper_target_position': 'gripper_target_position', 'gripper_actual_position': 'gripper_actual_position', 'gripper_initial_state': 'gripper_initial_state', 'grasp_attempt': 'grasp_attempt', 'gripper_state': 'gripper_state'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
