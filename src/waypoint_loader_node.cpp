#include <ros/ros.h>
#include "waypoint_system/waypoint_manager.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_loader_node");
  ros::NodeHandle nh("~");

  std::string waypoint_dir;
  nh.param<std::string>("waypoint_directory", waypoint_dir, "");

  if (waypoint_dir.empty()) {
    ROS_ERROR("[waypoint_loader_node] No waypoint_directory specified in parameters.");
    return 1;
  }

  waypoint_system::WaypointManager manager;
  if (!manager.loadWaypointsFromDirectory(waypoint_dir)) {
    ROS_ERROR("[waypoint_loader_node] Failed to load any waypoints from directory: %s", waypoint_dir.c_str());
    return 1;
  }

  const auto& paths = manager.getAllPaths();
  const auto& names = manager.getPathNames();

  ROS_INFO("[waypoint_loader_node] Loaded %lu paths.", paths.size());
  for (size_t i = 0; i < names.size(); ++i) {
    ROS_INFO("  [%lu] %s - %lu waypoints", i, names[i].c_str(), paths[i].size());
  }

  ros::spin();
  return 0;
}
