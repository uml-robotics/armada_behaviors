#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from armada_flexbe_utilities.msg import NamedPoseMoveAction, NamedPoseMoveGoal
from armada_flexbe_utilities.msg import CartesianMoveAction, CartesianMoveGoal

from geometry_msgs.msg import Pose

class WorkspaceSnapshotActionState(EventState):
        '''

        ># target_pose_list                 List of poses, this can be names (pre-defined, named poses) or waypoints (

        <= finished                         Arm successfully moved to named position
        <= failed                           Motion planning/execution failed

        '''

        def __init__(self):
                # See example_state.py for basic explanations.
                super(WorkspaceSnapshotActionState, self).__init__(outcomes = ['finished', 'failed'],
                                                               input_keys = ['target_pose_list'],
                                                               output_keys = ['concatenated_pointcloud'])

                # Setup action server client proxies
                self._topic = 'move_to_named_pose'
                self._client = ProxyActionClient({self._named_pose_move_action_topic: _topic})

                # Check for services and setup client proxies
                self._get_pointcloud_srv = rospy.ServiceProxy('/get_pointcloud', GetPointCloud)
                self._concatenate_pointcloud_srv = rospy.ServiceProxy('/concatenate_pointcloud', ConcatenatePointCloud)
                self._passthrough_filter_srv = rospy.ServiceProxy('/passthrough_filter', PointCloudPassthroughFilter)
                self._sac_segmentation_srv = rospy.ServiceProxy('/sac_segmentation', SacSegmentation)

                # It may happen that the action client fails to send the action goal.
                self._error = False


        def execute(self, userdata):
                # create the goal object for move_to_named_pose action
                goal = NamedPoseMoveGoal()

                for i in len(userdata.target_pose_list):
                    # Set current named pose goal
                    goal.pose_names = userdata.target_pose_list[i]
                    # Reset error state
                    self._error = False
                    # Attempt to send goal
                    try:
                        self._client.send_goal(self._topic, goal)
                    except Exception as e:
                        Logger.logwarn('Failed to send the command:\n%s' % str(e))
                        self._error = True
                    # Exit state on failed action goal request
                    if self._error:
                        return 'failed'
                    if self._client.has_result(self._topic):
                        result = self._client.get_result(self._topic)
                        execution_success = result.execution_success
                        # Check if action client failed to complete and exit state if it did
                        if execution_success == 0:
                            return 'failed'
                    # get pointcloud from current position and add
                    self._get_pointcloud_srv()

                # Concatenate snapshot pointclouds, apply passthrough filter and then segment out ground plane
                self._concatenate_pointcloud_srv()
                self._passthrough_filter_srv()
                self._sac_segmentation_srv()

                return 'finished'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                rospy.wait_for_service('/get_pointcloud')
                rospy.wait_for_service('/concatenate_pointcloud')
                rospy.wait_for_service('/passthrough_filter')
                rospy.wait_for_service('/sac_segmentation')

        def on_exit(self, userdata):
                # Make sure that the action is not running when leaving this state.
                if not self._client.has_result(self._named_pose_move_action_topic):
                    self._client.cancel(self._named_pose_move_action_topic)
                    Logger.loginfo('Cancelled active action goal.')
