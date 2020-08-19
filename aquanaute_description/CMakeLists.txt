cmake_minimum_required(VERSION 2.8.3)
project(aquanaute_description)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
  roslaunch
)

roslaunch_add_file_check(launch)

catkin_package()

install(
  DIRECTORY launch models urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

