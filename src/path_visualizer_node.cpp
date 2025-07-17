#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "waypoint_system/waypoint_manager.hpp"
#include <cmath>

#include <map>

// HSV 기반 상태 색상 분산
std_msgs::ColorRGBA getColorForState(int mission_state) {
  std_msgs::ColorRGBA color;
  color.a = 1.0;

  float hue = fmod(mission_state * 36.0f, 360.0f);
  float c = 1.0f;
  float x = 1.0f - std::abs(fmod(hue / 60.0f, 2.0f) - 1.0f);

  if (hue < 60)      { color.r = c; color.g = x; color.b = 0; }
  else if (hue < 120){ color.r = x; color.g = c; color.b = 0; }
  else if (hue < 180){ color.r = 0; color.g = c; color.b = x; }
  else if (hue < 240){ color.r = 0; color.g = x; color.b = c; }
  else if (hue < 300){ color.r = x; color.g = 0; color.b = c; }
  else               { color.r = c; color.g = 0; color.b = x; }

  return color;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_visualizer_node");
  ros::NodeHandle nh("~");

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypoint_markers", 1, true);

  std::string waypoint_dir;
  nh.param<std::string>("waypoint_directory", waypoint_dir, "");
  if (waypoint_dir.empty()) {
    ROS_ERROR("[path_visualizer_node] No waypoint_directory specified.");
    return 1;
  }

  waypoint_system::WaypointManager manager;
  if (!manager.loadWaypointsFromDirectory(waypoint_dir)) {
    ROS_ERROR("[path_visualizer_node] Failed to load waypoints.");
    return 1;
  }

  const auto& all_paths = manager.getAllPaths();
  const auto& names = manager.getPathNames();
  if (all_paths.empty()) {
    ROS_WARN("[path_visualizer_node] No paths loaded.");
    return 0;
  }

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "map";

  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;

  for (size_t path_idx = 0; path_idx < all_paths.size(); ++path_idx) {
    const auto& path = all_paths[path_idx];
    for (size_t i = 0; i < path.size(); ++i) {
      const auto& wp = path[i];

      // path (nav_msgs::Path)
      geometry_msgs::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose = wp.pose;
      path_msg.poses.push_back(ps);

      // index 텍스트 마커
      visualization_msgs::Marker text_marker;
      text_marker.header = path_msg.header;
      text_marker.ns = "text";
      text_marker.id = marker_id++;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.pose = wp.pose;
      text_marker.pose.position.z += 1.0; // float above point
      text_marker.scale.z = 0.6;
      text_marker.color = getColorForState(wp.mission_state);
      text_marker.text = std::to_string(wp.index);
      marker_array.markers.push_back(text_marker);

      // 점 마커 (컬러로 상태 구분)
      visualization_msgs::Marker point_marker;
      point_marker.header = path_msg.header;
      point_marker.ns = "point";
      point_marker.id = marker_id++;
      point_marker.type = visualization_msgs::Marker::SPHERE;
      point_marker.action = visualization_msgs::Marker::ADD;
      point_marker.pose = wp.pose;
      point_marker.scale.x = 0.3;
      point_marker.scale.y = 0.3;
      point_marker.scale.z = 0.3;
      point_marker.color = getColorForState(wp.mission_state);
      marker_array.markers.push_back(point_marker);
    }
  }

  path_pub.publish(path_msg);
  marker_pub.publish(marker_array);

  ROS_INFO("[path_visualizer_node] Published %lu waypoints and %lu markers.",
           path_msg.poses.size(), marker_array.markers.size());

  ros::spin();
  return 0;
}
