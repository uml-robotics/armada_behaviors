#!/usr/bin/env python
import rospy
import random

from flexbe_core import EventState, Logger
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


class SpawnModelServiceState(EventState):
        '''
        Example for a state to demonstrate which functionality is available for state implementation.
        This example lets the behavior wait until the given target_time has passed since the behavior has been started.

        -- model_name                   string          The desired name of the actual spawned object within the scene
        -- object_file_path             string          File path to the object .sdf file
        -- robot_namespace              string          Namespace the robot occupeis (if applicable)
        -- reference_frame              string          Reference frame from which the object should be spawned (e.g. world)

        <= continue                                     spawned/deleted an object successfully
        <= failed                                       something went wrong

        '''

        def __init__(self, model_name, object_file_path, robot_namespace, reference_frame):
                # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
                super(SpawnModelServiceState, self).__init__(outcomes = ['continue', 'failed'])

                # store object spawn pose info from previous state
                self._model_name = model_name
                self._object_file_path = object_file_path
                self._robot_namespace = robot_namespace
                self._reference_frame = reference_frame
                self._object_pose_x_min = rospy.get_param("/sim_objects/position/x_min")
                self._object_pose_x_max = rospy.get_param("/sim_objects/position/x_max")
                self._object_pose_y_min = rospy.get_param("/sim_objects/position/y_min")
                self._object_pose_y_max = rospy.get_param("/sim_objects/position/y_max")
                self._object_pose_z = rospy.get_param("/sim_objects/position/z")

        def execute(self, userdata):
                # This method is called periodically while the state is active.
                # Main purpose is to check state conditions and trigger a corresponding outcome.
                # If no outcome is returned, the state will stay active.

                # generate "random" [x,y,z] spawn coordinates within predefined bounds
                pose_x = round(random.uniform(self._object_pose_x_min, self._object_pose_x_max), 2)
                pose_y = round(random.uniform(self._object_pose_y_min, self._object_pose_y_max), 2)
                pose_z = self._object_pose_z

                # turn xyz coordinates into a Pose msg
                object_pose = Pose()
                object_pose.orientation.w = 1.0
                object_pose.position.x = pose_x
                object_pose.position.y = pose_y
                object_pose.position.z = pose_z

                # spawn_object service needs the full xml text from the model file
                xml_file = open(self._object_file_path,'r')
                model_xml = xml_file.read()

                spawn_model_srv = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

                try:
                  spawn_model_srv(self._model_name, model_xml, self._robot_namespace, object_pose, self._reference_frame)
                  return 'continue'
                except:
                  return 'failed'

        def on_enter(self, userdata):
                # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
                # It is primarily used to start actions which are associated with this state.

                # Logger.loginfo('attempting to spawn object...' )
                pass # Nothing to do in this state.

        def on_exit(self, userdata):
                # This method is called when an outcome is returned and another state gets active.
                # It can be used to stop possibly running processes started by on_enter.

                pass # Nothing to do in this state.

        def on_start(self):
                # This method is called when the behavior is started.
                # If possible, it is generally better to initialize used resources in the constructor
                # because if anything failed, the behavior would not even be started.

                rospy.wait_for_service('gazebo/spawn_sdf_model')

        def on_stop(self):
                # This method is called whenever the behavior stops execution, also if it is cancelled.
                # Use this event to clean up things like claimed resources.

                pass # Nothing to do in this state.

