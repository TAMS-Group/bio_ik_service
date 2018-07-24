project(bio_ik_service_examples)

cmake_minimum_required(VERSION 3.5)

find_package(catkin REQUIRED COMPONENTS
  rospy
  bio_ik_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  bio_ik_msgs
)
