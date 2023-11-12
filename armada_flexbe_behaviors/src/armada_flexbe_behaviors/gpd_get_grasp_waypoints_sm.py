#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_gpd_states.gpd_grasp_candidates_service_state import GPDGraspCandidatesServiceState as flexbe_gpd_states__GPDGraspCandidatesServiceState
from flexbe_gpd_states.gpd_grasp_waypoints_service_state import GPDGraspWaypointsServiceState as flexbe_gpd_states__GPDGraspWaypointsServiceState
from flexbe_pointcloud_states.pointcloud_publisher_state import PointCloudPublisherState as flexbe_pointcloud_states__PointCloudPublisherState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 01 2023
@author: bf
'''
class gpd_get_grasp_waypointsSM(Behavior):
	'''
	get grasp candidates list from gpd service and then solve for approach, target, and retreat waypoint poses
	'''


	def __init__(self):
		super(gpd_get_grasp_waypointsSM, self).__init__()
		self.name = 'gpd_get_grasp_waypoints'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:442 y:323, x:432 y:119
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_pointcloud = []
		_state_machine.userdata.grasp_candidates = []
		_state_machine.userdata.grasp_waypoints_list = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:150 y:81
			OperatableStateMachine.add('PublishTargetObjectPointCloud',
										flexbe_pointcloud_states__PointCloudPublisherState(topic=/topic),
										transitions={'continue': 'GPDGraspCandidates', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'target_pointcloud'})

			# x:149 y:352
			OperatableStateMachine.add('GPDGraspWaypoints',
										flexbe_gpd_states__GPDGraspWaypointsServiceState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_candidates': 'grasp_candidates', 'grasp_waypoints_list': 'grasp_waypoints_list'})

			# x:17 y:164
			OperatableStateMachine.add('WaitForGPDNodeRespawn',
										WaitState(wait_time=1),
										transitions={'done': 'PublishTargetObjectPointCloud'},
										autonomy={'done': Autonomy.Off})

			# x:147 y:261
			OperatableStateMachine.add('GPDGraspCandidates',
										flexbe_gpd_states__GPDGraspCandidatesServiceState(),
										transitions={'continue': 'GPDGraspWaypoints', 'failed': 'WaitForGPDNodeRespawn'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'combined_pointcloud': 'target_pointcloud', 'grasp_candidates': 'grasp_candidates'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
