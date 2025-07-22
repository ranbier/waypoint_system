#include "waypoint_system/waypoint_manager.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <limits>
#include <cmath>

namespace fs = std::filesystem;

namespace waypoint_system {

bool WaypointManager::loadWaypointsFromDirectory(const std::string& directory) {
  all_paths_.clear();
  path_names_.clear();

  if (!fs::exists(directory) || !fs::is_directory(directory)) {
    std::cerr << "[WaypointManager] Invalid directory: " << directory << std::endl;
    return false;
  }

  for (const auto& entry : fs::directory_iterator(directory)) {
    if (entry.path().extension() == ".csv") {
      std::vector<Waypoint> path = loadCsvFile(entry.path().string());
      if (!path.empty()) {
        all_paths_.emplace_back(path);
        path_names_.emplace_back(entry.path().filename().string());
      }
    }
  }

  return !all_paths_.empty();
}

std::vector<Waypoint> WaypointManager::loadCsvFile(const std::string& filepath) {
  std::ifstream file(filepath);
  std::vector<Waypoint> path;

  if (!file.is_open()) {
    std::cerr << "[WaypointManager] Failed to open: " << filepath << std::endl;
    return path;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string cell;

    Waypoint wp;

    try {
      std::getline(ss, cell, ',');
      wp.index = std::stoi(cell);

      std::getline(ss, cell, ',');
      wp.pose.position.x = std::stod(cell);

      std::getline(ss, cell, ',');
      wp.pose.position.y = std::stod(cell);

      std::getline(ss, cell, ',');
      wp.mission_state = std::stoi(cell);
    } catch (...) {
      continue;  // skip malformed line
    }

    wp.pose.position.z = 0.0;
    wp.velocity = 0.0;
    wp.heading = 0.0;
    wp.curvature = 0.0;

    path.push_back(wp);
  }

  return path;
}

int WaypointManager::findClosestWaypoint(const geometry_msgs::Pose& pose) const {
  if (all_paths_.empty()) return -1;

  const auto& path = all_paths_[0];
  int closest_index = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.size(); ++i) {
    double dx = pose.position.x - path[i].pose.position.x;
    double dy = pose.position.y - path[i].pose.position.y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = static_cast<int>(i);
    }
  }

  return closest_index;
}

std::vector<Waypoint> WaypointManager::extractLocalPath(int start_index, int size) const {
  std::vector<Waypoint> local;

  if (all_paths_.empty()) return local;
  const auto& path = all_paths_[0];

  int end = std::min(static_cast<int>(path.size()), start_index + size);
  for (int i = start_index; i < end; ++i) {
    local.push_back(path[i]);
  }

  return local;
}

Waypoint WaypointManager::getWaypoint(int index) const {
  if (all_paths_.empty()) return Waypoint();

  const auto& path = all_paths_[0];
  if (index >= 0 && index < static_cast<int>(path.size())) {
    return path[index];
  }
  return Waypoint();
}

const std::vector<std::vector<Waypoint>>& WaypointManager::getAllPaths() const {
  return all_paths_;
}

const std::vector<std::string>& WaypointManager::getPathNames() const {
  return path_names_;
}

}  // namespace waypoint_system
