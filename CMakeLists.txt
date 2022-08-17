cmake_minimum_required(VERSION 2.8.3)
project(handsfree_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY demo driver robot_para rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Install ##
#############

install(PROGRAMS scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES udev/10-imu.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

install(FILES udev/10-imu.rules
  DESTINATION /etc/udev/rules.d)
