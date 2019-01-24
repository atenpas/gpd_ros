# This message describes a 2-finger grasp configuration by its 6-DOF pose,
# consisting of a 3-DOF position and 3-DOF orientation, and the opening
# width of the robot hand.

# Position
geometry_msgs/Point position # grasp position (bottom/base center of robot hand)

# Orientation represented as three axis (R = [approach binormal axis])
geometry_msgs/Vector3 approach # grasp approach direction
geometry_msgs/Vector3 binormal # hand closing direction
geometry_msgs/Vector3 axis # hand axis

std_msgs/Float32 width # Required aperture (opening width) of the robot hand

std_msgs/Float32 score # Score assigned to the grasp by the classifier

geometry_msgs/Point sample # point at which the grasp was found
