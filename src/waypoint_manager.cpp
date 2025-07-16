#include "waypoint_system/waypoint_manager.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

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

    // index
    std::getline(ss, cell, ',');
    wp.index = std::stoi(cell);

    // x
    std::getline(ss, cell, ',');
    wp.pose.position.x = std::stod(cell);

    // y
    std::getline(ss, cell, ',');
    wp.pose.position.y = std::stod(cell);

    // mission_state
    std::getline(ss, cell, ',');
    wp.mission_state = std::stoi(cell);

    // 나머지 필드는 기본값으로 설정
    wp.pose.position.z = 0.0;
    wp.velocity = 0.0;
    wp.heading = 0.0;
    wp.curvature = 0.0;

    path.push_back(wp);
  }

  return path;
}

const std::vector<std::vector<Waypoint>>& WaypointManager::getAllPaths() const {
  return all_paths_;
}

const std::vector<std::string>& WaypointManager::getPathNames() const {
  return path_names_;
}

} // namespace waypoint_system