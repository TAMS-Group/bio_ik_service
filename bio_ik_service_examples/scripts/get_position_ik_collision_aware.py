#!/usr/bin/env python
from __future__ import print_function
import rospy

import moveit_msgs.msg
import moveit_msgs.srv
import trajectory_msgs.msg
import geometry_msgs.msg

rospy.init_node("bio_ik_service_example")

rospy.wait_for_service("/bio_ik/get_position_ik")
get_position_ik = rospy.ServiceProxy("/bio_ik/get_position_ik", moveit_msgs.srv.GetPositionIK)

request = moveit_msgs.msg.PositionIKRequest()

request.group_name = "all"

request.timeout.secs = 1

request.avoid_collisions = True

request.ik_link_names.append("l_wrist_roll_link")
request.pose_stamped_vector.append(geometry_msgs.msg.PoseStamped())
request.pose_stamped_vector[-1].header.frame_id = "1"
request.pose_stamped_vector[-1].pose.position.x = 0.6
request.pose_stamped_vector[-1].pose.position.y = 0.2
# request.pose_stamped_vector[-1].pose.position.y = -0.2 # UNCOMMENT THIS TO CAUSE A COLLISION
request.pose_stamped_vector[-1].pose.position.z = 1.0
request.pose_stamped_vector[-1].pose.orientation.x = 0.0
request.pose_stamped_vector[-1].pose.orientation.y = 0.0
request.pose_stamped_vector[-1].pose.orientation.z = 0.0
request.pose_stamped_vector[-1].pose.orientation.w = 1.0

request.ik_link_names.append("r_gripper_r_finger_tip_link")
request.pose_stamped_vector.append(geometry_msgs.msg.PoseStamped())
request.pose_stamped_vector[-1].header.frame_id = "1"
request.pose_stamped_vector[-1].pose.position.x = 0.7
request.pose_stamped_vector[-1].pose.position.y = -0.25
request.pose_stamped_vector[-1].pose.position.z = 1.0
request.pose_stamped_vector[-1].pose.orientation.x = 0.0
request.pose_stamped_vector[-1].pose.orientation.y = 0.0
request.pose_stamped_vector[-1].pose.orientation.z = 0.0
request.pose_stamped_vector[-1].pose.orientation.w = 1.0

request.ik_link_names.append("r_gripper_l_finger_tip_link")
request.pose_stamped_vector.append(geometry_msgs.msg.PoseStamped())
request.pose_stamped_vector[-1].header.frame_id = "1"
request.pose_stamped_vector[-1].pose.position.x = 0.7
request.pose_stamped_vector[-1].pose.position.y = -0.2
request.pose_stamped_vector[-1].pose.position.z = 1.0
request.pose_stamped_vector[-1].pose.orientation.x = 0.0
request.pose_stamped_vector[-1].pose.orientation.y = 0.0
request.pose_stamped_vector[-1].pose.orientation.z = 0.0
request.pose_stamped_vector[-1].pose.orientation.w = 1.0

response = get_position_ik(request)

print(response)

display = moveit_msgs.msg.DisplayTrajectory()
display.trajectory_start = response.solution
display.trajectory.append(moveit_msgs.msg.RobotTrajectory())
display.trajectory[0].joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
display.trajectory[0].joint_trajectory.points[-1].time_from_start.secs = 0
display.trajectory[0].joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
display.trajectory[0].joint_trajectory.points[-1].time_from_start.secs = 1
display_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, latch=True, queue_size=10)
display_publisher.publish(display)
rospy.spin()
