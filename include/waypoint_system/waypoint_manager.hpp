// waypoint_manager.hpp
#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <waypoint_system/Waypoint.h>

namespace waypoint_system {

class WaypointManager {
public:
  WaypointManager() = default;

  // 주어진 디렉토리 경로에서 모든 .csv 파일 로드
  bool loadWaypointsFromDirectory(const std::string& directory);

  // 모든 경로 반환
  const std::vector<std::vector<waypoint_system::Waypoint>>& getAllPaths() const;

  // 경로 이름들 반환 (디버깅용)
  const std::vector<std::string>& getPathNames() const;

private:
  std::vector<std::vector<waypoint_system::Waypoint>> all_paths_;
  std::vector<std::string> path_names_;

  // index, x, y, mission_state 형식의 CSV 파일 파싱
  std::vector<waypoint_system::Waypoint> loadCsvFile(const std::string& filepath);
};

} // namespace waypoint_system
