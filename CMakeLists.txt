cmake_minimum_required(VERSION 3.0.2)
project(waypoint_system)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
)

add_message_files(
  FILES
  Waypoint.msg
  WaypointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_manager
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# WaypointManager 라이브러리 정의
add_library(waypoint_manager
  src/waypoint_manager.cpp
)
target_link_libraries(waypoint_manager ${catkin_LIBRARIES})
add_dependencies(waypoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# waypoint_loader_node
add_executable(waypoint_loader_node src/waypoint_loader_node.cpp)
target_link_libraries(waypoint_loader_node waypoint_manager ${catkin_LIBRARIES})
add_dependencies(waypoint_loader_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# path_visualizer_node
add_executable(path_visualizer_node src/path_visualizer_node.cpp)
target_link_libraries(path_visualizer_node waypoint_manager ${catkin_LIBRARIES})
add_dependencies(path_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
