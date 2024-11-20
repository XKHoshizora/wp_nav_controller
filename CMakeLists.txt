cmake_minimum_required(VERSION 3.0.2)
project(wp_nav_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  actionlib_msgs
  move_base_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# 添加服务文件
add_service_files(
  FILES
  NavigationCommand.srv
  SequenceCommand.srv
)

catkin_python_setup()

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 确保消息生成在其他target之前
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/wp_nav_controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)