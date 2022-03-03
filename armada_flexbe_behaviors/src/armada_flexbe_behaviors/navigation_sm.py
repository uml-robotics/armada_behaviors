#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.move_base_state import MoveBaseState as armada_flexbe_states__MoveBaseState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.subscriber_state import SubscriberState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D, PoseStamped, Pose
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Tue Feb 22 2022
@author: Kevin Yassini
'''
class NavigationSM(Behavior):
	'''
	Move the robot to a defined location from RVIZ.
	'''


	def __init__(self):
		super(NavigationSM, self).__init__()
		self.name = 'Navigation'

		# parameters of this behavior
		self.add_parameter('x', 0)
		self.add_parameter('y', 0)
		self.add_parameter('theta', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 131 41 
		# Selecting a nav_goal in RVIZ uses movebase to move the robot anyway, but this will at least block until a goal is selected in RVIZ.



	def create(self):
		# x:276 y:422, x:65 y:431
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = Pose2D(self.x, self.y, self.theta)

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:59 y:137
			OperatableStateMachine.add('RvizPoseSubscriber',
										SubscriberState(topic="/move_base_simple/goal", blocking=True, clear=True),
										transitions={'received': 'Log', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'poseStamped'})

			# x:431 y:122
			OperatableStateMachine.add('MoveBase',
										armada_flexbe_states__MoveBaseState(),
										transitions={'arrived': 'finished', 'failed': 'failed'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'poseStamped'})

			# x:246 y:114
			OperatableStateMachine.add('Log',
										LogKeyState(text="PoseStamped: {}", severity=Logger.REPORT_HINT),
										transitions={'done': 'MoveBase'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'poseStamped'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
