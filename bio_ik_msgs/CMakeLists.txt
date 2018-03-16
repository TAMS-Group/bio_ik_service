cmake_minimum_required(VERSION 2.8.3)
project(bio_ik_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  geometry_msgs
  moveit_msgs
  genmsg
)

add_message_files(
  FILES
  AvoidJointLimitsGoal.msg
  BalanceGoal.msg
  CenterJointsGoal.msg
  ConeGoal.msg
  DirectionGoal.msg
  IKRequest.msg
  IKResponse.msg
  JointVariableGoal.msg
  LineGoal.msg
  LookAtGoal.msg
  MaxDistanceGoal.msg
  MinDistanceGoal.msg
  MinimalDisplacementGoal.msg
  OrientationGoal.msg
  PoseGoal.msg
  PositionGoal.msg
  SideGoal.msg
  TouchGoal.msg
)

add_service_files(
  FILES
  GetIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs
)

catkin_package(
  DEPENDS message_runtime
)
