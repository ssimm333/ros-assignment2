cmake_minimum_required(VERSION 3.8)
project(rescue_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# BT node implementations as a shared library
add_library(rescue_bt_nodes SHARED
  src/bt_nodes.cpp
)
target_include_directories(rescue_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rescue_bt_nodes ${DEPS})

# Main mission executor
add_executable(mission_bt_node src/mission_bt_node.cpp)
target_include_directories(mission_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_bt_node ${DEPS})
target_link_libraries(mission_bt_node rescue_bt_nodes)

# Install targets
install(TARGETS rescue_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
