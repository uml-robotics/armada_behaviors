# This message contains three grasp poses (point and orientation) for grasping and picking an object.
# This message contains the location from which the manipulator should begin its approach, grasp
# the object, and end its retreat. These points may be compiled into a list for further planning.

# Position from which to begin approach
geometry_msgs/Pose pre

# Grasping point target on the object
geometry_msgs/Pose target

# Position where retreat ends
geometry_msgs/Pose post
