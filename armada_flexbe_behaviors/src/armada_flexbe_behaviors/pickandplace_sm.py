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
from armada_flexbe_states.clear_octomap_service_state import ClearOctomapServiceState
from armada_flexbe_states.get_pointcloud_service_state import GetPointCloudServiceState
from armada_flexbe_states.gpd_grasp_candidates_service_state import GPDGraspCandidatesServiceState
from armada_flexbe_states.gpd_grasp_waypoints_service_state import GPDGraspWaypointsServiceState
from armada_flexbe_states.gripper_command_action_state import GripperCommandActionState
from armada_flexbe_states.move_arm_action_state import MoveArmActionState
from armada_flexbe_states.pcl_concatenate_pointcloud_service_state import PCLConcatenatePointCloudServiceState
from armada_flexbe_states.pcl_euclidean_cluster_extraction_service_state import PCLEuclideanClusterExtractionServiceState
from armada_flexbe_states.pcl_passthrough_filter_service_state import PCLPassthroughFilterServiceState
from armada_flexbe_states.pcl_plane_segmentation_service_state import PCLPlaneSegmentationServiceState
from armada_flexbe_states.pointcloud_publisher_state import PointCloudPublisherState
from armada_flexbe_states.retreat_commander_state import RetreatCommanderState
from armada_flexbe_states.snapshot_commander_state import SnapshotCommanderState
from armada_flexbe_states.spawn_table_collision_service_state import SpawnTableCollisionServiceState
from flexbe_practice_states.step_iterator_state import stepIteratorState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Dec 7 2022
@author: Brian Flynn
'''
class PickAndPlaceSM(Behavior):
	'''
	Perform a pick and place option with a real robot arm and objects.
	'''


	def __init__(self):
		super(PickAndPlaceSM, self).__init__()
		self.name = 'PickAndPlace'

		# parameters of this behavior
		self.add_parameter('wait_time', 2)
		self.add_parameter('obstacle_cloud_topic', '/obstacle_cloud')
		self.add_parameter('concatenated_cloud_topic', 'combined_cloud')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:349 y:633, x:367 y:351
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.wait_pose = ['above']
		_state_machine.userdata.snapshot_pose_list = ['above','robot_left','robot_right']
		_state_machine.userdata.target_pose = ['']
		_state_machine.userdata.current_snapshot_step = 0
		_state_machine.userdata.pointcloud_list = []
		_state_machine.userdata.combined_pointcloud = 0
		_state_machine.userdata.grasp_candidates = []
		_state_machine.userdata.grasp_waypoints_list = []
		_state_machine.userdata.grasp_attempt = 0
		_state_machine.userdata.grasp_state = 'approach'
		_state_machine.userdata.dropoff_pose = ['dropoff']
		_state_machine.userdata.obstacles_pointcloud_list = []
		_state_machine.userdata.obstacles_pointcloud = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

        # [/MANUAL_CREATE]

		# x:468 y:396, x:475 y:216
		_sm_solvegraspwaypointscontainer_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['combined_pointcloud', 'grasp_candidates', 'obstacles_pointcloud_list', 'obstacles_pointcloud'], output_keys=['grasp_waypoints_list', 'obstacles_pointcloud_list'])

		with _sm_solvegraspwaypointscontainer_0:
			# x:152 y:26
			OperatableStateMachine.add('PCLConcatenateObstaclesPointCloud',
										PCLConcatenatePointCloudServiceState(),
										transitions={'continue': 'PublishObstaclesPointCloud', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list_in': 'obstacles_pointcloud_list', 'combined_pointcloud': 'obstacles_pointcloud', 'pointcloud_list_out': 'obstacles_pointcloud_list'})

			# x:167 y:468
			OperatableStateMachine.add('GPDGraspWaypoints',
										GPDGraspWaypointsServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_candidates': 'grasp_candidates', 'grasp_waypoints_list': 'grasp_waypoints_list'})

			# x:177 y:204
			OperatableStateMachine.add('PublishObjectsPointCloud',
										PointCloudPublisherState(topic=self.concatenated_cloud_topic),
										transitions={'continue': 'GPDGraspCandidates', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'combined_pointcloud'})

			# x:175 y:112
			OperatableStateMachine.add('PublishObstaclesPointCloud',
										PointCloudPublisherState(topic=self.obstacle_cloud_topic),
										transitions={'continue': 'PublishObjectsPointCloud', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'obstacles_pointcloud'})

			# x:54 y:285
			OperatableStateMachine.add('WaitForNodeRespawn',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'PublishObjectsPointCloud'},
										autonomy={'done': Autonomy.Off})

			# x:166 y:366
			OperatableStateMachine.add('GPDGraspCandidates',
										GPDGraspCandidatesServiceState(),
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
										GetPointCloudServiceState(),
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


		# x:389 y:384, x:415 y:158
		_sm_pclfiltercontainer_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pointcloud_list', 'combined_pointcloud', 'obstacles_pointcloud_list'], output_keys=['combined_pointcloud', 'pointcloud_list', 'obstacles_pointcloud_list'])

		with _sm_pclfiltercontainer_3:
			# x:30 y:40
			OperatableStateMachine.add('PCLConcatenatePointCloud',
										PCLConcatenatePointCloudServiceState(),
										transitions={'continue': 'PCLPassthroughFilter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list_in': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud', 'pointcloud_list_out': 'pointcloud_list'})

			# x:17 y:375
			OperatableStateMachine.add('PCLEuclideanClusterExtraction',
										PCLEuclideanClusterExtractionServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'obstacles_cloud_list_in': 'obstacles_pointcloud_list', 'target_cloud_out': 'combined_pointcloud', 'obstacles_cloud_list_out': 'obstacles_pointcloud_list'})

			# x:40 y:146
			OperatableStateMachine.add('PCLPassthroughFilter',
										PCLPassthroughFilterServiceState(),
										transitions={'continue': 'PCLPlaneSegmentation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:35 y:255
			OperatableStateMachine.add('PCLPlaneSegmentation',
										PCLPlaneSegmentationServiceState(),
										transitions={'continue': 'PCLEuclideanClusterExtraction', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'obstacles_cloud_list_in': 'obstacles_pointcloud_list', 'objects_cloud_out': 'combined_pointcloud', 'obstacles_cloud_list_out': 'obstacles_pointcloud_list'})


		# x:203 y:421, x:475 y:110
		_sm_approachcontainer_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grasp_waypoints_list', 'grasp_attempt'], output_keys=['grasp_attempt'])

		with _sm_approachcontainer_4:
			# x:30 y:40
			OperatableStateMachine.add('ApproachCommander',
										ApproachCommanderState(),
										transitions={'continue': 'MoveArmGrasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_task_candidates': 'grasp_waypoints_list', 'grasp_attempt': 'grasp_attempt', 'target_pose_list': 'target_pose_list'})

			# x:212 y:116
			OperatableStateMachine.add('GraspStepIterator',
										stepIteratorState(),
										transitions={'continue': 'ApproachCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'iterator_in': 'grasp_attempt', 'iterator_out': 'grasp_attempt'})

			# x:140 y:274
			OperatableStateMachine.add('GripperClose',
										GripperCommandActionState(gripper_target_position=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:30 y:176
			OperatableStateMachine.add('MoveArmGrasp',
										MoveArmActionState(),
										transitions={'finished': 'GripperClose', 'failed': 'GraspStepIterator'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'target_pose_list'})



		with _state_machine:
			# x:39 y:73
			OperatableStateMachine.add('SpawnTableCollision',
										SpawnTableCollisionServiceState(),
										transitions={'continue': 'MoveArmPreSnapshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:759 y:341
			OperatableStateMachine.add('ClearOctomap',
										ClearOctomapServiceState(),
										transitions={'continue': 'MoveArmPreSnapshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:528 y:563
			OperatableStateMachine.add('GripperCommandOpen',
										GripperCommandActionState(gripper_target_position=0),
										transitions={'continue': 'MoveArmHome', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:546 y:481
			OperatableStateMachine.add('MoveArmDropoff',
										MoveArmActionState(),
										transitions={'finished': 'GripperCommandOpen', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'dropoff_pose'})

			# x:549 y:648
			OperatableStateMachine.add('MoveArmHome',
										MoveArmActionState(),
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'wait_pose'})

			# x:545 y:401
			OperatableStateMachine.add('MoveArmPostGrasp',
										MoveArmActionState(),
										transitions={'finished': 'MoveArmDropoff', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose_list': 'wait_pose'})

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
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud', 'obstacles_pointcloud_list': 'obstacles_pointcloud_list'})

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

			# x:37 y:336
			OperatableStateMachine.add('SolveGraspWaypointsContainer',
										_sm_solvegraspwaypointscontainer_0,
										transitions={'finished': 'ApproachContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'combined_pointcloud': 'combined_pointcloud', 'grasp_candidates': 'grasp_candidates', 'obstacles_pointcloud_list': 'obstacles_pointcloud_list', 'obstacles_pointcloud': 'obstacles_pointcloud', 'grasp_waypoints_list': 'grasp_waypoints_list'})

			# x:58 y:437
			OperatableStateMachine.add('ApproachContainer',
										_sm_approachcontainer_4,
										transitions={'finished': 'RetreatContainer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grasp_waypoints_list': 'grasp_waypoints_list', 'grasp_attempt': 'grasp_attempt'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

    # [/MANUAL_FUNC]
