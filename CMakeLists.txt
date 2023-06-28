cmake_minimum_required(VERSION 2.8.3)
project(handsfree_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/HandsfreeRosImu.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY demo launch rviz udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/create_udev_rules
  scripts/display_3D_visualization.py
  scripts/get_imu_rpy.py
  scripts/hfi_a9_ros.py
  scripts/hfi_b6_ros.py
  scripts/hfi_b9_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES udev/10-imu.rules
  DESTINATION /etc/udev/rules.d)
