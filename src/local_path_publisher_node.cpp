#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <waypoint_system/waypoint_manager.hpp>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <string>
#include <waypoint_maker/State.h>
#include <waypoint_maker/Lane.h>
#include <waypoint_maker/Waypoint.h>
//0310차선보간
#include <new_lane/lidar_topic_msg.h>
//0310차선보간
#include <std_msgs/Float64.h>


namespace wp = waypoint_system;

class LocalPathPublisher {
public:
  LocalPathPublisher(ros::NodeHandle& nh)
    : nh_(nh) {

    nh_.param("waypoint_directory", waypoint_directory_, std::string(""));
    nh_.param("path_publish_size_", path_publish_size_, 50);
    nh_.param("loop_path", loop_path, false);
    nh_.param("jump_dist_m_", jump_dist_m_, 6.0);
    nh_.param("trend_window_", trend_window, 8);
    nh_.param("extrapolate_ds_m_", extrapolate_ds_m, 0.0);
    nh_.param("gap_good_required_", gap_good_required, 3);
    nh_.param("gap_bad_required_", gap_bad_required, 2);
    // 0306 차선보간 수정 시작
    nh_.param("path_switch_blend_steps_", path_switch_blend_steps_, 10);
    nh_.param("path_switch_blend_max_points_", path_switch_blend_max_points_, 80);
    // if (path_switch_blend_max_points_ < 0) path_switch_blend_max_points_ = 0;
    // if (path_switch_blend_steps_ < 0) path_switch_blend_steps_ = 0;
    // 0306 차선보간 수정 끝
    nh_.param("lane_change_done_th_", lane_change_done_th_, 0.6);


    manager_.setJumpDistM(jump_dist_m_);
    manager_.setTrendWindow(trend_window);
    manager_.setExtrapolateDsM(extrapolate_ds_m);
    manager_.setGapHysteresis(gap_good_required, gap_bad_required);

    ROS_INFO("[local_path_publisher] jump_dist_m_=%.2f", jump_dist_m_);
    ROS_INFO("[local_path_publisher] trend_window_=%d extrapolate_ds_m_=%.2f gap_good_required_=%d gap_bad_required_=%d",
         trend_window, extrapolate_ds_m, gap_good_required, gap_bad_required);
    // 0306 차선보간 수정 시작
    ROS_INFO("[local_path_publisher] path_switch_blend_steps_=%d", path_switch_blend_steps_);
    ROS_INFO("[local_path_publisher] path_switch_blend_max_points_=%d", path_switch_blend_max_points_);
    // 0306 차선보간 수정 끝

    if (!manager_.loadWaypointsFromDirectory(waypoint_directory_)) {
      ROS_ERROR("[local_path_publisher] Failed to load waypoint CSV files.");
      ros::shutdown();
    }
    manager_.setLoopEnabled(loop_path);

    //0310차선보간
    lidar_topic_sub_ = nh_.subscribe("/lidar_topic", 1, &LocalPathPublisher::lidarTopicCallback, this);
    //0310차선보간

    pose_sub_ = nh_.subscribe("/odom", 10, &LocalPathPublisher::odomCallback, this);
    path_number_sub_ = nh_.subscribe("/path_number", 1, &LocalPathPublisher::pathNumberCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    current_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("/current_pose_marker", 1);
    trajectory_path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);
    gps_state_pub_ = nh_.advertise<waypoint_maker::State>("/gps_state", 1);
    final_waypoint_pub_ = nh_.advertise<waypoint_maker::Lane>("/final_waypoint", 1, true);
    // change_dist_pub_ = nh_.advertise<std_msgs::Float64>("/change_dist", 1);



    trajectory_path_.header.frame_id = "map";
  }

private:
  // 0306 차선보간 수정 시작
  std::vector<wp::Waypoint> blendLocalPathLinear(const std::vector<wp::Waypoint>& from_path,
                                                 const std::vector<wp::Waypoint>& to_path,
                                                 double alpha) const {
    if (to_path.empty()) return {};

    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    std::vector<wp::Waypoint> blended;
    blended.reserve(to_path.size());

    // 0306 차선보간 수정 시작
    const size_t full_size = std::min(from_path.size(), to_path.size());
    const size_t max_size = (path_switch_blend_max_points_ > 0)
                                ? static_cast<size_t>(path_switch_blend_max_points_)
                                : full_size;  // 0이면 전체 보간
    const size_t blend_size = std::min(full_size, max_size);
    // 0306 차선보간 수정 끝
    for (size_t i = 0; i < blend_size; ++i) {
      wp::Waypoint wp = to_path[i];
      wp.pose.position.x = (1.0 - alpha) * from_path[i].pose.position.x + alpha * to_path[i].pose.position.x;
      wp.pose.position.y = (1.0 - alpha) * from_path[i].pose.position.y + alpha * to_path[i].pose.position.y;
      // wp.pose.position.z = (1.0 - alpha) * from_path[i].pose.position.z + alpha * to_path[i].pose.position.z;
      blended.push_back(wp);
    }

    for (size_t i = blend_size; i < to_path.size(); ++i) {
      blended.push_back(to_path[i]);
    }

    return blended;
  }
  // 0306 차선보간 수정 끝

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::Pose current_pose = msg->pose.pose;
    int closest_index = manager_.findClosestWaypoint(current_pose);
    if (closest_index == -1) {
      ROS_WARN_THROTTLE(1.0, "[local_path_publisher] No closest waypoint found.");
      return;
    }

    // Terminal logging: current pose and closest waypoint info
    const wp::Waypoint closest_wp = manager_.getWaypoint(closest_index);
    // gps_state publish
    waypoint_maker::State st;
    st.lane_number   = manager_.getActivePathNumber();     // (=현재 path_number)
    st.current_state = closest_wp.mission_state;           // (=현재 mission_state)
    // st.dist          = computeDistToNextState(closest_index, st.current_state); // optional
    gps_state_pub_.publish(st); 

    
  // 로컬 경로 추출 : 전체 경로에서 로봇 바로 앞에 있는 일부분만 잘라낸다. 
  // closeset_index : 현재 위치에서 가장 가까운 웨이포인트의 인덱스
  // path_publish_size : 앞으로 몇개의 점을 보여줄지 결정하는 크기   
  // closest_index에서 부터 path_publish_size만큼 잘라서 local_path로 publish               
    std::vector<wp::Waypoint> local_path = manager_.extractLocalPath(closest_index, path_publish_size_);
    // 0306 차선보간 수정 시작

    //0310차선보간 acc_active_
    if (!acc_active_ &&
        path_switch_blending_active_ &&
        path_switch_blend_steps_ > 0 &&
        !path_switch_from_path_.empty() &&
        !local_path.empty()) {
      ++path_switch_blend_step_now_;

      const double alpha =
          static_cast<double>(path_switch_blend_step_now_) /
          static_cast<double>(path_switch_blend_steps_);
      local_path = blendLocalPathLinear(path_switch_from_path_, local_path, alpha);

      if (path_switch_blend_step_now_ >= path_switch_blend_steps_) {
        path_switch_blending_active_ = false;
        path_switch_from_path_.clear();
      }
    }
    // 0306 차선보간 수정 끝

    bool using_new_path = false;
    for (const auto& p : local_path) {
      if (p.index == -1) { using_new_path = true; break; }
    }

    std::string closest_str = using_new_path ? "new" : std::to_string(closest_index);

    // === /final_waypoint publish (local_path 기반) ===
    waypoint_maker::Lane lane_msg;
    lane_msg.header.frame_id = "map";
    lane_msg.header.stamp = ros::Time::now();
    lane_msg.waypoints.reserve(local_path.size());

    int lane_num = manager_.getActivePathNumber();

    for (int i = 0; i < (int)local_path.size(); ++i) {
      const auto& p = local_path[i];

      waypoint_maker::Waypoint w;
      w.waypoint_index = (p.index >= 0) ? p.index : i;   // extrapolate(-1)일 때 대체 인덱스
      w.pose.header = lane_msg.header;

      // wp::Waypoint::pose 는 geometry_msgs::Pose 이므로 이렇게 넣으면 됨
      w.pose.pose = p.pose;

      w.mission_state = p.mission_state;
      w.lane_number   = lane_num;

      w.ryaw = p.heading;
      w.rk = p.curvature;

      lane_msg.waypoints.push_back(w);
    }

    final_waypoint_pub_.publish(lane_msg);


    if (using_new_path) {
      ROS_WARN_THROTTLE(0.5,
        "[local_path_publisher] pose=(%.3f, %.3f)  closest_idx=%s  wp=(%.3f, %.3f)",
        current_pose.position.x, current_pose.position.y,
        closest_str.c_str(),
        closest_wp.pose.position.x, closest_wp.pose.position.y);
    } else {
      ROS_INFO_THROTTLE(0.5,
        "[local_path_publisher] pose=(%.3f, %.3f)  closest_idx=%s  wp=(%.3f, %.3f)",
        current_pose.position.x, current_pose.position.y,
        closest_str.c_str(),
        closest_wp.pose.position.x, closest_wp.pose.position.y);
    }

//     ROS_INFO("[local_path_publisher] pose=(%.3f, %.3f)  closest_idx=%d  wp=(%.3f, %.3f)",
//                       current_pose.position.x,
//                       current_pose.position.y,
//                       closest_index,
//                       closest_wp.pose.position.x,
//                       closest_wp.pose.position.y);

    

//  // === [ADD] extrapolate(연장) 구간 사용 여부 로깅 ===
//     int fake_cnt = 0;
//     int first_fake_i = -1;
//     for (int i = 0; i < (int)local_path.size(); ++i) {
//       if (local_path[i].index == -1) {
//         fake_cnt++;
//         if (first_fake_i < 0) first_fake_i = i;
//       }
//     }

//     if (!local_path.empty()) {
//       const auto& last = local_path.back();
//       // 0.5초에 한 번만 찍히게(터미널 도배 방지)
//       ROS_INFO_THROTTLE(0.5,
//         "[local_path_publisher][extrapolate-check] closest_idx=%d local_size=%zu fake_cnt=%d first_fake_i=%d last_wp_idx=%d last_wp=(%.2f, %.2f)",
//         closest_index, local_path.size(), fake_cnt, first_fake_i, last.index,
//         last.pose.position.x, last.pose.position.y);
//     }








    nav_msgs::Path path_msg;        // nav_msgs Path 생성
    path_msg.header = msg->header;    //msg: odom

    for (const auto& wp : local_path) {
      geometry_msgs::PoseStamped pose_stamped;    //pose_stemped 객체 생성
      pose_stamped.header = path_msg.header;
      pose_stamped.pose = wp.pose;                // wp.pose는 position과 orientation의 정보가 담겨있다. 
      path_msg.poses.push_back(pose_stamped);     // position = x,y,z
                                                  // orientation = x,y,z,w -> 로봇이 위치에 도달했을 때 바라봐야할 방향
    }

    path_pub_.publish(path_msg);
    // 0306 차선보간 수정 시작
    last_published_local_path_ = local_path;
    // 0306 차선보간 수정 끝

    visualization_msgs::Marker arrow;
    arrow.header = msg->header;
    arrow.ns = "current_pose";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose = current_pose;

    arrow.scale.x = 3.0;  // 길이
    arrow.scale.y = 1.0;
    arrow.scale.z = 1.0;

    arrow.color.r = 0.2;
    arrow.color.g = 1.0;
    arrow.color.b = 0.2;
    arrow.color.a = 1.0;

    current_pose_pub_.publish(arrow);

    // 3. Trajectory Path 누적 및 publish
    geometry_msgs::PoseStamped current_stamped;
    current_stamped.header = msg->header;
    current_stamped.pose = current_pose;

    trajectory_path_.poses.push_back(current_stamped);
    trajectory_path_.header.stamp = ros::Time::now();
    trajectory_path_pub_.publish(trajectory_path_);
  }



  //0310차선보간
  void lidarTopicCallback(const new_lane::lidar_topic_msg::ConstPtr& msg) {
    acc_active_ = msg->ACC_flag;

    if (acc_active_ && path_switch_blending_active_) {
      path_switch_blending_active_ = false;
      path_switch_blend_step_now_ = 0;
      path_switch_from_path_.clear();
    }
  }
  //0310차선보간

  // lidar에서 바꿀 경로의 번호를 보내줌
  void pathNumberCallback(const std_msgs::Int32::ConstPtr& msg) {
    // manager_.setActivePathNumber(msg->data);
    // ROS_INFO("[local_path_publisher] Active path number changed to %d", msg->data);
    // 0306 차선보간 수정 시작
    const int current_path = manager_.getActivePathNumber();
    if (msg->data == current_path) return;

    if (!acc_active_ && path_switch_blend_steps_ > 0 && !last_published_local_path_.empty()) {
      path_switch_from_path_ = last_published_local_path_;
      path_switch_blending_active_ = true;
      path_switch_blend_step_now_ = 0;
    } else {
      path_switch_blending_active_ = false;
      path_switch_from_path_.clear();
    }

    manager_.setActivePathNumber(msg->data);
    // ROS_INFO("[local_path_publisher] Active path number changed to %d", msg->data);
    // 0306 차선보간 수정 끝
  }

  // //dist 사용
  // double computeDistToNextState(int closest_index, int cur_state) {
  //   const auto& paths = manager_.getAllPaths();
  //   int pn = manager_.getActivePathNumber();
  //   if (pn < 0 || pn >= (int)paths.size()) return -1.0;

  //   const auto& path = paths[pn];
  //   if (path.empty() || closest_index < 0 || closest_index >= (int)path.size()) return -1.0;

  //   // loop_path=true일 때 wrap을 고려해서 최대 path.size()만큼만 탐색
  //   double dist = 0.0;
  //   int prev = closest_index;

  //   for (int step = 1; step < (int)path.size(); ++step) {
  //     int idx = closest_index + step;

  //     if (!loop_path) {
  //       if (idx >= (int)path.size()) break;
  //     } else {
  //       idx %= (int)path.size();
  //     }

  //     // 거리 누적
  //     const auto& a = path[prev].pose.position;
  //     const auto& b = path[idx].pose.position;
  //     dist += std::hypot(b.x - a.x, b.y - a.y);

  //     // state 변화 감지
  //     if (path[idx].mission_state != cur_state) {
  //       return dist;
  //     }
  //     prev = idx;
  //   }
  //   return -1.0; // 다음 state 못 찾음
  // }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber path_number_sub_;
  //0310차선보간
  ros::Subscriber lidar_topic_sub_;
  //0310차선보간
  ros::Publisher path_pub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher trajectory_path_pub_;
  ros::Publisher gps_state_pub_;
  ros::Publisher final_waypoint_pub_;
  // ros::Publisher change_dist_pub_;

  nav_msgs::Path trajectory_path_;


  wp::WaypointManager manager_;
  std::string waypoint_directory_;
  int path_publish_size_;
  bool loop_path;
  double jump_dist_m_;
  int trend_window;
  double extrapolate_ds_m;
  int gap_good_required, gap_bad_required;

  bool lane_change_active_ = false;
  double lane_change_done_th_ = 0.6;   // 튜닝

    // 0306 차선보간 수정 시작
  int path_switch_blend_steps_ = 10;
  int path_switch_blend_max_points_ = 80;
  bool path_switch_blending_active_ = false;
  int path_switch_blend_step_now_ = 0;
  // 0310차선보간
  bool acc_active_ = false;
  //0310차선보간
  std::vector<wp::Waypoint> path_switch_from_path_;
  std::vector<wp::Waypoint> last_published_local_path_;
  // 0306 차선보간 수정 끝
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_path_publisher_node");
  ros::NodeHandle nh("~");

  LocalPathPublisher node(nh);
  ros::spin();
  return 0;
}
