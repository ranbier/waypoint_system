// waypoint_manager.hpp
#pragma once

#include <ros/ros.h>
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

  int findClosestWaypoint(const geometry_msgs::Pose& pose) const;
  int getLoopIndex(const std::vector<Waypoint>& path) const;

  std::vector<waypoint_system::Waypoint> extractLocalPath(int start_index, int size) const;

  waypoint_system::Waypoint getWaypoint(int index) const;

  // 모든 경로 반환
  const std::vector<std::vector<waypoint_system::Waypoint>>& getAllPaths() const;

  // 경로 이름들 반환 (디버깅용)
  const std::vector<std::string>& getPathNames() const;

  void setLoopEnabled(bool enabled);

  void setActivePathNumber(int number);

  void setTrendWindow(int w);

  void setExtrapolateDsM(double ds);

  void setGapHysteresis(int good_required, int bad_required);

  int getActivePathNumber() const;

  void setJumpDistM(double m);

private:
  std::vector<std::vector<waypoint_system::Waypoint>> all_paths_;
  std::vector<std::string> path_names_;
  bool loop_enabled_ = false;
  int path_number_ = 0;
  double jump_dist_m_ = 6.0;
  mutable bool runtime_extrapolate_ = false; 

  mutable int ex_closest_index = -1;

  int trend_window_ = 8;
  double extrapolate_ds_m_ = 0.0;   // 0이면 자동

  mutable int gap_good_cnt_ = 0;   // gap <= dist2 연속 <- config
  mutable int gap_bad_cnt_  = 0;   // gap >  dist2 연속

  int gap_good_required_ = 3;      // 정상루프 복귀 조건
  int gap_bad_required_  = 2;      // extrapolate 진입 조건

  // index, x, y, mission_state 형식의 CSV 파일 파싱
  std::vector<waypoint_system::Waypoint> loadCsvFile(const std::string& filepath);
};

} // namespace waypoint_system
