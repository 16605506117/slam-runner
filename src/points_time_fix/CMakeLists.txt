cmake_minimum_required(VERSION 3.0.2)
project(points_time_fix)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/time_fix.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
