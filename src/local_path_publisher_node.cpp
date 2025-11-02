#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <waypoint_system/waypoint_manager.hpp>
#include <tf/tf.h>
#include <std_msgs/Int32.h>

namespace wp = waypoint_system;

class LocalPathPublisher {
public:
  LocalPathPublisher(ros::NodeHandle& nh)
    : nh_(nh) {

    nh_.param("waypoint_directory", waypoint_directory_, std::string(""));
    nh_.param("path_publish_size", path_publish_size_, 50);
    nh_.param("loop_path", loop_path, false);

    if (!manager_.loadWaypointsFromDirectory(waypoint_directory_)) {
      ROS_ERROR("[local_path_publisher] Failed to load waypoint CSV files.");
      ros::shutdown();
    }
    manager_.setLoopEnabled(loop_path);

    pose_sub_ = nh_.subscribe("/odom", 10, &LocalPathPublisher::odomCallback, this);
    path_number_sub_ = nh_.subscribe("/path_number", 1, &LocalPathPublisher::pathNumberCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
    current_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("/current_pose_marker", 1);
    trajectory_path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);

    trajectory_path_.header.frame_id = "map";
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::Pose current_pose = msg->pose.pose;
  int closest_index = manager_.findClosestWaypoint(current_pose);
    if (closest_index == -1) {
      ROS_WARN_THROTTLE(1.0, "[local_path_publisher] No closest waypoint found.");
      return;
    }

    // Terminal logging: current pose and closest waypoint info
    const wp::Waypoint closest_wp = manager_.getWaypoint(closest_index);
    ROS_INFO_THROTTLE(0.1,
                      "[local_path_publisher] pose=(%.3f, %.3f)  closest_idx=%d  wp=(%.3f, %.3f)",
                      current_pose.position.x,
                      current_pose.position.y,
                      closest_index,
                      closest_wp.pose.position.x,
                      closest_wp.pose.position.y);

    std::vector<wp::Waypoint> local_path = manager_.extractLocalPath(closest_index, path_publish_size_);
    nav_msgs::Path path_msg;
    path_msg.header = msg->header;

    for (const auto& wp : local_path) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose = wp.pose;
      path_msg.poses.push_back(pose_stamped);
    }

    path_pub_.publish(path_msg);

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

  void pathNumberCallback(const std_msgs::Int32::ConstPtr& msg) {
  manager_.setActivePathNumber(msg->data);
    ROS_INFO("[local_path_publisher] Active path number changed to %d", msg->data);
  }

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber path_number_sub_;
  ros::Publisher path_pub_;
  ros::Publisher current_pose_pub_;
  ros::Publisher trajectory_path_pub_;

  nav_msgs::Path trajectory_path_;


  wp::WaypointManager manager_;
  std::string waypoint_directory_;
  int path_publish_size_;
  bool loop_path;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_path_publisher_node");
  ros::NodeHandle nh("~");

  LocalPathPublisher node(nh);
  ros::spin();
  return 0;
}
