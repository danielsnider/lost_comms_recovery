cmake_minimum_required(VERSION 2.8.3)

project(lost_comms_recovery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  move_base_msgs
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy actionlib actionlib_msgs sensor_msgs geometry_msgs move_base_msgs
)

catkin_install_python(
  PROGRAMS nodes/lost_comms_recovery
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

