#include "waypoint_system/waypoint_manager.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <limits>
#include <cmath>
// #include <algorithm>

namespace fs = std::filesystem;
using namespace std;

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

  const auto& path = all_paths_[path_number_];
  const int path_size = static_cast<int>(path.size());
  if (path_size == 0) return -1;

  int closest_index = -1;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < path_size; ++i) {
    double dx = pose.position.x - path[i].pose.position.x;
    double dy = pose.position.y - path[i].pose.position.y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
    
  }
  
  if (ex_closest_index >= 0 && ex_closest_index < path_size && (closest_index != path_size -1)) {
    int diff = std::abs(closest_index - ex_closest_index);
    if (loop_enabled_) diff = std::min(diff, path_size - diff); // 원형 거리
  
    if (diff > 50) {
      return ex_closest_index;
    }
  
  } else if (loop_enabled_ && closest_index == path_size - 1) {
    closest_index = getLoopIndex(path);
    cout << "Looping to index: " << closest_index << endl;
  }
  ex_closest_index = closest_index;

  return closest_index;
}

int WaypointManager::getLoopIndex(const std::vector<Waypoint>& path) const {

  const int path_size = static_cast<int>(path.size());
  if (path_size == 0) return -1;

  int closest_index = -1;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < path_size / 2; ++i) {
      double dx = path[path_size - 1].pose.position.x - path[i].pose.position.x;
      double dy = path[path_size - 1].pose.position.y - path[i].pose.position.y;
      double d2 = dx * dx + dy * dy;
      if (d2 < min_dist) {
        min_dist = d2;
        closest_index = i;
      }
    }

    return closest_index;
}


// Progressive search and auto-wrap logic removed to restore original behavior

std::vector<Waypoint> WaypointManager::extractLocalPath(int start_index, int size) const {
  std::vector<Waypoint> local;
  if (all_paths_.empty()) return local;

  const auto& path = all_paths_[path_number_];
  int path_size = static_cast<int>(path.size());

  for (int i = 0; i < size; ++i) {
    int idx = start_index + i;
    if (loop_enabled_) {
      idx = idx % path_size;
    } else if (idx >= path_size) {
      break;
    }
    local.push_back(path[idx]);
  }

  return local;
}

Waypoint WaypointManager::getWaypoint(int index) const {
  if (all_paths_.empty()) return Waypoint();

  const auto& path = all_paths_[path_number_];
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

void WaypointManager::setLoopEnabled(bool enabled) {
  loop_enabled_ = enabled;
}

void WaypointManager::setActivePathNumber(int number) {
  if (number >= 0 && number < all_paths_.size()) {
    path_number_ = number;
  } else {
    ROS_WARN("[WaypointManager] Invalid path number: %d", number);
  }
}

int WaypointManager::getActivePathNumber() const {
  return path_number_;
}

}  // namespace waypoint_system
