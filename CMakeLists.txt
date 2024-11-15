cmake_minimum_required(VERSION 3.0.2)
project(wp_nav_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  geometry_msgs
  tf
  amr_map_tools
  message_generation
  std_msgs
)

add_message_files(
  FILES
  NavigationCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_navigator
  CATKIN_DEPENDS
    roscpp
    actionlib
    move_base_msgs
    geometry_msgs
    tf
    amr_map_tools
    message_runtime
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 核心库
add_library(waypoint_navigator
  src/waypoint_navigator.cpp
  src/navigation_status.cpp
)
target_link_libraries(waypoint_navigator
  ${catkin_LIBRARIES}
)
add_dependencies(waypoint_navigator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 节点可执行文件
add_executable(wp_nav_controller_node src/wp_nav_controller_node.cpp)
target_link_libraries(wp_nav_controller_node
  waypoint_navigator
  ${catkin_LIBRARIES}
)
add_dependencies(wp_nav_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 安装规则
install(TARGETS waypoint_navigator wp_nav_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)