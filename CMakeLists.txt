cmake_minimum_required(VERSION 2.8.3)
project(handsfree_ros_imu)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
)

generate_dynamic_reconfigure_options(
  cfg/HandsfreeRosImu.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hfi_a9_ros_node src/hfi_a9_ros_node.cpp)
add_dependencies(hfi_a9_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hfi_a9_ros_node ${catkin_LIBRARIES})

add_executable(hfi_b6_ros_node src/hfi_b6_ros_node.cpp)
add_dependencies(hfi_b6_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hfi_b6_ros_node ${catkin_LIBRARIES})

add_executable(hfi_b9_ros_node src/hfi_b9_ros_node.cpp)
add_dependencies(hfi_b9_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hfi_b9_ros_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS hfi_a9_ros_node hfi_b6_ros_node hfi_b9_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY demo launch rviz udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/create_udev_rules
  scripts/display_3D_visualization.py
  scripts/get_imu_rpy.py
  scripts/hfi_a9_ros.py
  scripts/hfi_b6_ros.py
  scripts/hfi_b9_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES udev/10-imu.rules
  DESTINATION /etc/udev/rules.d)
