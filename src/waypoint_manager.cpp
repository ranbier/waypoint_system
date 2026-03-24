#include "waypoint_system/waypoint_manager.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <limits>
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;
using namespace std;

namespace waypoint_system {

// bool WaypointManager::loadWaypointsFromDirectory(const std::string& directory) {
//   all_paths_.clear();
//   path_names_.clear();

//   if (!fs::exists(directory) || !fs::is_directory(directory)) {
//     std::cerr << "[WaypointManager] Invalid directory: " << directory << std::endl;
//     return false;
//   }

//   for (const auto& entry : fs::directory_iterator(directory)) {
//     if (entry.path().extension() == ".csv") {
//       std::vector<Waypoint> path = loadCsvFile(entry.path().string());
//       if (!path.empty()) {
//         all_paths_.emplace_back(path);                                    // csv의 모든 경로 all_paths에 넣기
//         path_names_.emplace_back(entry.path().filename().string());       // csv 파일의 이름(filename().string())은 path_names_에 넣기 
//       }
//     }
//   }

//   return !all_paths_.empty();
// }

// 0203 수정
bool WaypointManager::loadWaypointsFromDirectory(const std::string& directory) {
  all_paths_.clear();
  path_names_.clear();

  if (!fs::exists(directory) || !fs::is_directory(directory)) {
    std::cerr << "[WaypointManager] Invalid directory: " << directory << std::endl;
    return false;
  }

  // 1) csv 파일 경로들을 먼저 모은다
  std::vector<fs::path> csv_files;
  for (const auto& entry : fs::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      csv_files.push_back(entry.path());
    }
  }

  // 2) 파일명 기준으로 정렬 (0.csv, 1.csv, 2.csv ...)
  std::sort(csv_files.begin(), csv_files.end(),
            [](const fs::path& a, const fs::path& b) {
              return a.filename().string() < b.filename().string();
            });

  // 3) 정렬된 순서대로 로드
  for (const auto& p : csv_files) {
    std::vector<Waypoint> path = loadCsvFile(p.string());
    if (!path.empty()) {
      all_paths_.emplace_back(std::move(path));
      path_names_.emplace_back(p.filename().string());
    }
  }

  return !all_paths_.empty();
}
// 0203 수정

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
      
      std::getline(ss, cell, ',');
      wp.heading = std::stod(cell);

      std::getline(ss, cell, ',');
      wp.curvature = std::stod(cell);
    } catch (...) {
      continue;  // skip malformed line
    }

    wp.pose.position.z = 0.0;
    wp.velocity = 0.0;
    // wp.heading = 0.0;
    // wp.curvature = 0.0;

    path.push_back(wp);
  }

  return path;
}

void WaypointManager::setJumpDistM(double m) {
  if (m > 0.1) jump_dist_m_ = m;
}

void WaypointManager::setTrendWindow(int w) {
  if (w < 2) w = 2;
  if (w > 500) w = 500;   // 과도하게 크면 의미 없음(원하면 조절)
  trend_window_ = w;
}

void WaypointManager::setExtrapolateDsM(double ds) {
  // 0이면 자동, 양수면 고정 스텝
  if (ds < 0.0) ds = 0.0;
  extrapolate_ds_m_ = ds;
}

void WaypointManager::setGapHysteresis(int good_required, int bad_required) {
  if (good_required < 1) good_required = 1;
  if (bad_required  < 1) bad_required  = 1;
  gap_good_required_ = good_required;
  gap_bad_required_  = bad_required;
}

static double wrapToPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

int WaypointManager::findClosestWaypoint(const geometry_msgs::Pose& pose) const {
  if (all_paths_.empty()) return -1;

  const auto& path = all_paths_[path_number_];
  const int path_size = static_cast<int>(path.size());
  // runtime_extrapolate_ = false; 

  if (path_size == 0) return -1;

  int closest_index = -1;
  double min_dist = std::numeric_limits<double>::max();   // numeric_limits -> double 형 중 가장 큰 값
  for (int i = 0; i < path_size; ++i) {   // path_size : 인덱스 size
    double dx = pose.position.x - path[i].pose.position.x;
    double dy = pose.position.y - path[i].pose.position.y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      closest_index = i;
    }
  }
  if (closest_index < 0) return -1;
  
  // if (ex_closest_index >= 0 && ex_closest_index < path_size && (closest_index != path_size -1)) 
  // {
  //   int diff = std::abs(closest_index - ex_closest_index);    // abs -> 절댓값


    
  //   if (loop_enabled_)  // 원형으로 이어진 경로
  //   {
  //   diff = std::min(diff, path_size - diff);    // diff : 그냥 직진 사이의 거리 index 차이, 
  //                                               // path_size - diff : 반대로 돌아서 가는 거리 index 차이
  //   }

  //   if (diff > 50) {
  //     return ex_closest_index;
  //   }
  
  // }

  // // 최종 closest_index 일 때 -> diff 없음 -> 수정 완료
  // else if (loop_enabled_ && closest_index == path_size - 1) {
  //   closest_index = getLoopIndex(path);
  //   cout << "Looping to index: " << closest_index << endl;
  // }


  // 2) closest 점프 방지(기존 diff>50 대체)
  const double th2 = jump_dist_m_ * jump_dist_m_;

  if (ex_closest_index >= 0 && ex_closest_index < path_size) {
    const double jdx = path[closest_index].pose.position.x - path[ex_closest_index].pose.position.x;
    const double jdy = path[closest_index].pose.position.y - path[ex_closest_index].pose.position.y;
    const double jump_d2 = jdx * jdx + jdy * jdy;

    // if (jump_d2 > th2) {
    //   return ex_closest_index;
    // }
    if (jump_d2 > th2) {
    // ★ 예외: 현재 pose가 "새 후보 closest_index" 근처면(실제로 도착한 거면) 점프 허용
    const double cdx = pose.position.x - path[closest_index].pose.position.x;
    const double cdy = pose.position.y - path[closest_index].pose.position.y;
    const double cand_d2 = cdx * cdx + cdy * cdy;

    // cand_d2 기준은 별도 파라미터로 빼도 되는데, 일단 th2를 재사용해도 동작함
    // 더 타이트하게 하고 싶으면 (예: 2.0m)로 고정 추천
    const double reacquire_d2 = 2.0 * 2.0;

    if (cand_d2 > reacquire_d2) {
      return ex_closest_index;  // 아직 도착한 게 아니면 기존대로 점프 방지
    }

    // 도착했으면: 이 순간부터는 원래 path로 복귀해야 하니까 extrapolate 상태도 초기화
    runtime_extrapolate_ = false;
    gap_good_cnt_ = 0;
    gap_bad_cnt_ = 0;
    // return 안 하고 아래로 계속 진행해서 closest_index=0/1이 그대로 채택되게 둠
    }
  }

  // 3) loop 마지막점 처리 
  if (loop_enabled_ && closest_index == path_size - 1) {
    const int loop_idx = getLoopIndex(path);

    bool gap_good = false;
    if (loop_idx >= 0 && loop_idx < path_size) {
      const double dx = path[path_size - 1].pose.position.x - path[loop_idx].pose.position.x;
      const double dy = path[path_size - 1].pose.position.y - path[loop_idx].pose.position.y;
      const double gap2 = dx * dx + dy * dy;

      //th2(config) 보다 작으면 gap good
      gap_good = (gap2 <= th2);
    }
    else {
      gap_good = false; // 후보 없으면 bad
    }

    // 카운터 업데이트(연속성)
    if (gap_good) {
      gap_good_cnt_++;
      gap_bad_cnt_ = 0;
    }
    else {
      gap_bad_cnt_++;
      gap_good_cnt_ = 0;
    }

    // 상태 전환(플래핑 방지)
    if (runtime_extrapolate_) {
      // extrapolate -> 정상 복귀는 더 보수적으로
      if (gap_good_cnt_ >= gap_good_required_) {
        runtime_extrapolate_ = false;
      }
    }
    else {
      // 정상 -> extrapolate 진입
      if (gap_bad_cnt_ >= gap_bad_required_) {
        runtime_extrapolate_ = true;
      }
    }

    // loop 점프는 "정상모드 && gap_good"일 때만
    if (!runtime_extrapolate_ && gap_good && loop_idx >= 0 && loop_idx < path_size) {
      closest_index = loop_idx;
      std::cout << "Looping to index: " << closest_index << std::endl;
    }

  }
  
  else {
    // last 상황이 아니면 extrapolate 의미가 없으니 리셋
    runtime_extrapolate_ = false;
    gap_good_cnt_ = 0;
    gap_bad_cnt_ = 0;
  }

  // 4) ex 업데이트
  ex_closest_index = closest_index;
  return closest_index;
}


// 항상 거리로 가장가까운 점을 고른다(첫 절반에서부터) -> 겹칠 수도 있음 
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

// 최근 window개의 segment로 yaw(방향), kappa(곡률), ds(간격) 추정
static void estimateTrend(const std::vector<waypoint_system::Waypoint>& path,
                          int window,
                          double& yaw0,
                          double& kappa,
                          double& ds_out)
{
  int seg_cnt = 0;

  const int n = static_cast<int>(path.size());

  // --- 안전 가드: 점이 부족하면 기본값/단순값으로 처리 ---
  if (n < 2) {
    yaw0 = 0.0;
    kappa = 0.0;
    ds_out = 0.5;
    return;
  }

  // 점 2개면 segment 1개 -> yaw는 가능, kappa는 0
  if (n == 2) {
    const double dx = path[1].pose.position.x - path[0].pose.position.x;
    const double dy = path[1].pose.position.y - path[0].pose.position.y;
    yaw0 = std::atan2(dy, dx);

    ds_out = std::hypot(dx, dy);
    // if (ds_out < 0.2) ds_out = 0.2;   // ds_out 값 csv에서 확인  ***
    // if (ds_out > 2.0) ds_out = 2.0;

    if (ds_out < 0.02) ds_out = 0.02;
    if (ds_out > 3.0)  ds_out = 3.0;

    kappa = 0.0;
    return;
  }

  // n >= 3부터는 yaw 평균 + 곡률 추정 가능
  const int max_window = n - 1;                // segment 개수
  window = std::max(2, std::min(window, max_window)); // 최소 2 segment

  const int start = (n - 1) - window;

  double sum_sin = 0.0, sum_cos = 0.0;
  double sum_ds = 0.0;

  double sum_kappa = 0.0;
  int kcnt = 0;

  double prev_yaw = 0.0;
  bool has_prev = false;

  for (int i = start; i < n - 1; ++i) {
    const double dx = path[i + 1].pose.position.x - path[i].pose.position.x;
    const double dy = path[i + 1].pose.position.y - path[i].pose.position.y;
    const double ds = std::hypot(dx, dy);
    if (ds < 1e-3) continue;

    seg_cnt++;

    const double yaw = std::atan2(dy, dx);

    // 헤딩 평균(각도는 sin/cos 평균)
    sum_sin += std::sin(yaw);
    sum_cos += std::cos(yaw);
    sum_ds += ds;

    // 곡률 추정( yaw 변화율 / 거리 )
    if (has_prev) {
      const double dyaw = wrapToPi(yaw - prev_yaw);
      sum_kappa += (dyaw / ds);
      kcnt++;
    }
    prev_yaw = yaw;
    has_prev = true;
  }

  // 유효 segment가 하나도 없으면(전부 ds<1e-3), 마지막 segment로 fallback
  if (std::abs(sum_sin) < 1e-12 && std::abs(sum_cos) < 1e-12) {
    const double dx = path[n - 1].pose.position.x - path[n - 2].pose.position.x;
    const double dy = path[n - 1].pose.position.y - path[n - 2].pose.position.y;
    yaw0 = std::atan2(dy, dx);
  }
  else {
    yaw0 = std::atan2(sum_sin, sum_cos);
  }

  ds_out = (seg_cnt > 0) ? (sum_ds / seg_cnt) : 0.5;
  kappa = (kcnt > 0) ? (sum_kappa / kcnt) : 0.0;

  // ***
  // 안전 클램프(튜닝 가능)
  // if (ds_out < 0.2) ds_out = 0.2;
  // if (ds_out > 2.0) ds_out = 2.0;

  if (ds_out < 0.02) ds_out = 0.02;
  if (ds_out > 3.0)  ds_out = 3.0;

  const double kappa_max = 0.3; // 1/m (반경 ~3.3m)
  if (kappa >  kappa_max) kappa =  kappa_max;
  if (kappa < -kappa_max) kappa = -kappa_max;
}

// Progressive search and auto-wrap logic removed to restore original behavior
std::vector<Waypoint> WaypointManager::extractLocalPath(int start_index, int size) const {
  std::vector<Waypoint> local;
  if (all_paths_.empty()) return local;

  const auto& path = all_paths_[path_number_];
  const int path_size = static_cast<int>(path.size());
  if (path_size == 0) return local;

  // 안전장치: start_index가 음수면 빈 벡터 반환(노드 쪽에서도 막는 게 더 좋음)
  if (start_index < 0) return local;

  // 1) loop_enabled_ == false : 연장 X, 끝에서 끊기
  if (!loop_enabled_) {
    for (int i = 0; i < size; ++i) {
      const int idx = start_index + i;
      if (idx >= path_size) break;
      local.push_back(path[idx]);
    }
    return local;
  }

  // 2) loop_enabled_ == true && runtime_extrapolate_ == false : 정상 루프 넘겨주기
  if (!runtime_extrapolate_) {
    for (int i = 0; i < size; ++i) {
      int idx = start_index + i;
      idx = idx % path_size;
      local.push_back(path[idx]);
    }
    return local;
  }

  // 3) loop_enabled_ == true && runtime_extrapolate_ == true : wrap 금지 + 끝에서 연장

  // 3-1) path size 작으면 넘겨주기
  if (path_size < 2) {
    // 점이 1개면 연장 방향이 없으니 그 점만 반복(또는 break 해도 됨)
    for (int i = 0; i < size; ++i) local.push_back(path[0]);
    return local;
  }

  // 최근 N개로 경향성(yaw, kappa, ds) 추정
  // const int trend_window = 8; // 5~10 추천
  double yaw0 = 0.0, kappa = 0.0, ds_est = 0.5;
  estimateTrend(path, trend_window_, yaw0, kappa, ds_est);

  // ds 최종 결정: config로 고정값이 들어오면 그걸 쓰고, 아니면 자동 추정값(ds_est)
  double ds = (extrapolate_ds_m_ > 1e-6) ? extrapolate_ds_m_ : ds_est;

  // ds 안전 클램프(원하면 ds_est 안에서만 해도 됨)
  
  // ***
  //morai, erp42에 csv의 ds 따라서 수정해야함
  // if (ds < 0.2) ds = 0.2;
  // if (ds > 2.0) ds = 2.0;

  // ds_est 기반 상대 클램프 (CSV 간격을 따라가게)
  const double ds_min_rel = std::max(0.02, 0.5 * ds_est);
  const double ds_max_rel = std::min(3.0,  2.0 * ds_est);

  if (ds < ds_min_rel) ds = ds_min_rel;
  if (ds > ds_max_rel) ds = ds_max_rel;

  // (선택) 최후의 절대 안전 클램프: 비정상 CSV/값 방어용
  // if (ds < 0.02) ds = 0.02;
  // if (ds > 3.0)  ds = 3.0;

  const Waypoint& p1 = path[path_size - 1];

  // 연장 구간 누적 상태
  double x = p1.pose.position.x;
  double y = p1.pose.position.y;
  double yaw = yaw0;
  bool extrap_started = false;

  for (int i = 0; i < size; ++i) {
    const int idx = start_index + i;

    // 아직 원본 점이 남아 있으면 그대로 사용
    if (idx < path_size) {
      local.push_back(path[idx]);
      continue;
    }

    // 연장 시작 시점에서 초기화(한 번만)
    if (!extrap_started) {
      x = p1.pose.position.x;
      y = p1.pose.position.y;
      yaw = yaw0;
      extrap_started = true;
    }

    // 상수 곡률 모델로 한 스텝 전진
    x += ds * std::cos(yaw);
    y += ds * std::sin(yaw);
    yaw = wrapToPi(yaw + kappa * ds);

    Waypoint wp = p1;
    wp.index = -1; // 가짜 점 표시
    wp.pose.position.x = x;
    wp.pose.position.y = y;

    local.push_back(wp);
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

// lidar에서 바꿀 경로의 번호를 보내줌 -> number!

// 여기서 path_number에 gps 기준 현재 차선이 있어야됨
void WaypointManager::setActivePathNumber(int number) {
  if (number >= 0 && number < all_paths_.size()) {

    // 같은 번호면 건드리지 않는다
    if (path_number_ == number) return;

    // 만약 다른 번호일때는 강제로 path_number_를 변경해서 find closest, extract local path, getwaypoint까지 한다.
    path_number_ = number;
    
    //경로가 바뀌면 이전 closest 히스토리를 초기화 시킨다.
    ex_closest_index = -1;
    runtime_extrapolate_ = false;
    gap_good_cnt_ = 0;
    gap_bad_cnt_ = 0;

    ROS_INFO("[WaypointManager] Active path changed to: %d", number);
  } else {
    ROS_WARN("[WaypointManager] Invalid path number: %d", number);
  }
}

int WaypointManager::getActivePathNumber() const {
  return path_number_;
}

}  // namespace waypoint_system
