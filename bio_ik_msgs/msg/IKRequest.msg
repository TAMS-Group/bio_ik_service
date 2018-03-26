# Name of the rosparam parameter from which to load the robot model
# default: "robot_description"
string robot_description

# Name of the move group for which to compute IK
# Only joints that are part of the specified move group will be modified by the IK solver
string group_name

# If no accurate solution could be found and approximate is false,
# no solution will be returned. If approximate is true, the best
# approximate solution will be returned, even if no accurate solution could be found.
bool approximate

# IK timeout
# If no solution could be found within the specified timeout, the IK request is canceled
# and if return_approximate_solution is false, no solution is returned.
duration timeout

# Maximum number of IK attempts
# (For BioIK, it's usually best to set this to 1, or to not set this at all if the default
# value in the kinematics.yaml is set to 1, and, if necessary, to increase the timeout instead.
# Unless you've enabled avoid_collisions. In cluttered environments and with avoid_collisions enabled,
# you may want to increase the number of attempts until you get a sufficiently high overall success rate.)
int32 attempts

# Joints specified as fixed_joints will always keep their initial positions and will
# not be moved by the IK solver
string[] fixed_joints

# Initial robot state from which to start looking for IK solutions
moveit_msgs/RobotState robot_state

# If true, IK results will be checked for collisions, and IK results that result in collisions
# will be rejected.
# For reliably finding IK solutions in cluttered environments, you may also want to increase the
# number of attempts.
# Collision checking requires a moveit planning scene to be present. If no planning scene was found
# the bio_ik_service node will print a ROS_ERROR message on startup or during the first collision
# aware IK request.
bool avoid_collisions

# Typed goal lists
PositionGoal[] position_goals
OrientationGoal[] orientation_goals
PoseGoal[] pose_goals
LookAtGoal[] look_at_goals
MinDistanceGoal[] min_distance_goals
MaxDistanceGoal[] max_distance_goals
LineGoal[] line_goals
TouchGoal[] touch_goals
AvoidJointLimitsGoal[] avoid_joint_limits_goals
CenterJointsGoal[] center_joints_goals
MinimalDisplacementGoal[] minimal_displacement_goals
JointVariableGoal[] joint_variable_goals
BalanceGoal[] balance_goals
SideGoal[] side_goals
DirectionGoal[] direction_goals
ConeGoal[] cone_goals
