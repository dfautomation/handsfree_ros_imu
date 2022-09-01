cmake_minimum_required(VERSION 2.8.3)
project(handsfree_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS
  system
  chrono
  thread
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/create_udev_rules
  scripts/hfi_b6_ros.py
  scripts/hfi_a9_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES udev/10-imu.rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

install(FILES udev/10-imu.rules
  DESTINATION /etc/udev/rules.d)
