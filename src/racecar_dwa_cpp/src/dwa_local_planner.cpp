#include <chrono>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// NEW: yaw -> quaternion 변환용
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class DwaLocalPlanner : public rclcpp::Node
{
public:
  DwaLocalPlanner()
  : Node("dwa_local_planner")
  {
    // 파라미터 선언 (기본값)
    max_vel_x_        = declare_parameter("max_vel_x",        3.0);
    min_vel_x_        = declare_parameter("min_vel_x",        0.0);
    max_yaw_rate_     = declare_parameter("max_yaw_rate",     2.0);   // [rad/s]
    acc_x_            = declare_parameter("acc_x",            3.0);   // [m/s^2]
    acc_yaw_          = declare_parameter("acc_yaw",          3.0);   // [rad/s^2]
    v_res_            = declare_parameter("v_res",            0.2);
    w_res_            = declare_parameter("w_res",            0.2);
    dt_               = declare_parameter("dt",               0.1);
    predict_time_     = declare_parameter("predict_time",     2.0);

    weight_heading_   = declare_parameter("weight_heading",   1.0);
    weight_path_      = declare_parameter("weight_path",      1.0);
    weight_obstacle_  = declare_parameter("weight_obstacle",  1.0);
    weight_velocity_  = declare_parameter("weight_velocity",  0.5);

    robot_radius_     = declare_parameter("robot_radius",     0.3);
    safety_distance_  = declare_parameter("safety_distance",  0.3);
    wheel_base_       = declare_parameter("wheel_base",       0.33);
    look_ahead_dist_  = declare_parameter("look_ahead_dist",  1.0);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10,
      std::bind(&DwaLocalPlanner::odomCallback, this, std::placeholders::_1));

    // ★ 여기 QoS는 나중에 SensorDataQoS로 바꾸는 게 좋음 (지금은 그대로 둠)
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&DwaLocalPlanner::scanCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/optimal_path", 10,
      std::bind(&DwaLocalPlanner::pathCallback, this, std::placeholders::_1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);

    // NEW: 로컬 플랜 퍼블리셔
    local_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/dwa_local_plan", 10);

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&DwaLocalPlanner::controlLoop, this));

    RCLCPP_INFO(get_logger(), "DWA local planner node started.");
  }

private:
  // ===== 콜백들 =====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;

    // 현재 속도 업데이트 (Twist 기준, 단순 unicycle 가정)
    last_v_ = msg->twist.twist.linear.x;
    last_w_ = msg->twist.twist.angular.z;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = *msg;
    has_scan_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = *msg;
    has_path_ = !msg->poses.empty();
  }

  // ===== 메인 제어 루프 =====
  void controlLoop()
  {
    if (!has_odom_ || !has_scan_ || !has_path_) {
      // 데이터 준비 안 됐으면 정지 명령
      publishStop();
      return;
    }

    Pose2D current_pose = odomToPose2D(latest_odom_);

    double best_score = -std::numeric_limits<double>::infinity();
    double best_v = 0.0;
    double best_w = 0.0;

    // dynamic window 계산
    double vs_min = std::max(min_vel_x_, last_v_ - acc_x_ * dt_);
    double vs_max = std::min(max_vel_x_, last_v_ + acc_x_ * dt_);
    double ws_min = std::max(-max_yaw_rate_, last_w_ - acc_yaw_ * dt_);
    double ws_max = std::min( max_yaw_rate_, last_w_ + acc_yaw_ * dt_);

    // v, w 샘플링
    for (double v = vs_min; v <= vs_max + 1e-6; v += v_res_) {
      for (double w = ws_min; w <= ws_max + 1e-6; w += w_res_) {
        // 너무 느리고 회전도 거의 없는 경우는 스킵
        if (std::fabs(v) < 0.05 && std::fabs(w) < 0.05) {
          continue;
        }

        // trajectory 예측
        double path_cost = 0.0;
        double heading_cost = 0.0;
        double obstacle_cost = 0.0;

        bool collision = false;
        Pose2D final_pose = simulateTrajectory(
          current_pose, v, w, path_cost, heading_cost, obstacle_cost, collision);

        if (collision) {
          continue;  // 충돌 나는 후보는 버림
        }

        double velocity_cost = v; // 속도 자체를 보상 (크면 좋음)

        double score =
          weight_path_     * (-path_cost) +      // path_cost는 작을수록 좋으니 -붙임
          weight_heading_  * (-heading_cost) +
          weight_obstacle_ * (-obstacle_cost) +  // obstacle_cost도 작을수록 좋게 정의
          weight_velocity_ * (velocity_cost);

        if (score > best_score) {
          best_score = score;
          best_v = v;
          best_w = w;
        }
      }
    }

    // 적당한 후보가 없으면 정지
    if (!std::isfinite(best_score)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No valid DWA candidate found, stopping.");
      publishStop();
      return;
    }

    // NEW: 여기서 best_v, best_w에 대해 로컬 trajectory 다시 적분해서 Path로 퍼블리시
    publishBestLocalPlan(current_pose, best_v, best_w);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "DWA cmd v=%.2f, w=%.2f, score=%.2f",
      best_v, best_w, best_score);

    publishDrive(best_v, best_w);
  }

  // ===== Trajectory 시뮬레이션 =====
  Pose2D simulateTrajectory(
    const Pose2D & start,
    double v, double w,
    double & path_cost_out,
    double & heading_cost_out,
    double & obstacle_cost_out,
    bool & collision_out)
  {
    Pose2D pose = start;

    const int steps = static_cast<int>(predict_time_ / dt_);
    collision_out = false;

    double min_dist_to_path = std::numeric_limits<double>::infinity();
    obstacle_cost_out = 0.0;

    for (int i = 0; i < steps; ++i) {
      // simple unicycle 모델
      pose.x   += v * std::cos(pose.yaw) * dt_;
      pose.y   += v * std::sin(pose.yaw) * dt_;
      pose.yaw += w * dt_;

      // 장애물과의 거리 체크 (scan 사용, 매우 러프한 방식)
      double obs_cost = obstacleCostScan();

      if (obs_cost > obstacle_cost_out) {
        obstacle_cost_out = obs_cost; // trajectory 전체 중 최악 비용
      }

      // 전역 경로와 거리
      double d = distanceToPath(pose);
      if (d < min_dist_to_path) {
        min_dist_to_path = d;
      }
    }

    // heading cost: 최종 pose에서 look-ahead point 방향과 yaw 차이
    heading_cost_out = headingCost(pose);
    path_cost_out = min_dist_to_path;
    return pose;
  }

  // NEW: best (v, w)에 대한 로컬 trajectory를 Path로 변환해서 퍼블리시
  void publishBestLocalPlan(const Pose2D & start, double v, double w)
  {
    const int steps = static_cast<int>(predict_time_ / dt_);

    std::vector<Pose2D> traj;
    traj.reserve(steps + 1);

    Pose2D pose = start;
    traj.push_back(pose);

    for (int i = 0; i < steps; ++i) {
      pose.x   += v * std::cos(pose.yaw) * dt_;
      pose.y   += v * std::sin(pose.yaw) * dt_;
      pose.yaw += w * dt_;
      traj.push_back(pose);
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    // odom frame 기준으로 그려주는 게 제일 안전
    if (!latest_odom_.header.frame_id.empty()) {
      path_msg.header.frame_id = latest_odom_.header.frame_id;
    } else {
      path_msg.header.frame_id = "ego_racecar/odom";
    }

    for (const auto & p : traj) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, p.yaw);
      ps.pose.orientation = tf2::toMsg(q);

      path_msg.poses.push_back(ps);
    }

    local_plan_pub_->publish(path_msg);
  }

  // ===== Path 관련 유틸 =====
  double distanceToPath(const Pose2D & pose)
  {
    if (latest_path_.poses.empty()) {
      return 0.0;
    }

    double min_dist = std::numeric_limits<double>::infinity();

    for (const auto & ps : latest_path_.poses) {
      double dx = pose.x - ps.pose.position.x;
      double dy = pose.y - ps.pose.position.y;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < min_dist) {
        min_dist = d;
      }
    }
    return min_dist;
  }

  double headingCost(const Pose2D & pose)
  {
    if (latest_path_.poses.empty()) {
      return 0.0;
    }

    // 현재 pose에서 look_ahead_dist_ 만큼 떨어진 path point 찾기
    const auto & path = latest_path_.poses;
    double min_diff = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::PoseStamped target;

    for (size_t i = 0; i < path.size(); ++i) {
      double dx = path[i].pose.position.x - pose.x;
      double dy = path[i].pose.position.y - pose.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (std::fabs(dist - look_ahead_dist_) < min_diff) {
        min_diff = std::fabs(dist - look_ahead_dist_);
        target = path[i];
      }
    }

    double dx = target.pose.position.x - pose.x;
    double dy = target.pose.position.y - pose.y;
    double target_yaw = std::atan2(dy, dx);
    double diff = normalizeAngle(target_yaw - pose.yaw);

    return std::fabs(diff);  // 작을수록 좋음
  }

  double obstacleCostScan()
  {
    if (!has_scan_) return 0.0;

    double min_range = std::numeric_limits<double>::infinity();
    for (double r : latest_scan_.ranges) {
      if (std::isnan(r) || std::isinf(r)) continue;
      if (r < min_range) min_range = r;
    }

    if (min_range < robot_radius_ + safety_distance_) {
      return 100.0;
    }

    // 거리가 멀수록 좋으니까 1/min_range 형태로 비용 정의
    return 1.0 / (min_range + 1e-3);
  }

  // ===== Ackermann 명령 발행 =====
  void publishDrive(double v, double w)
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "ego_racecar/base_link"; // 대충 프레임명

    msg.drive.speed = v;

    // 간단한 ackermann 변환 (wheel_base_ 이용)
    double steering = 0.0;
    if (std::fabs(v) > 1e-3) {
      steering = std::atan2(w * wheel_base_, v);
    }
    msg.drive.steering_angle = steering;

    last_v_ = v;
    last_w_ = w;

    drive_pub_->publish(msg);
  }

  void publishStop()
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "ego_racecar/base_link";
    msg.drive.speed = 0.0;
    msg.drive.steering_angle = 0.0;
    drive_pub_->publish(msg);
    last_v_ = 0.0;
    last_w_ = 0.0;
  }

  // ===== 유틸 함수 =====
  Pose2D odomToPose2D(const nav_msgs::msg::Odometry & odom)
  {
    Pose2D p;
    p.x = odom.pose.pose.position.x;
    p.y = odom.pose.pose.position.y;

    const auto & q = odom.pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    p.yaw = std::atan2(siny_cosp, cosy_cosp);

    return p;
  }

  double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

private:
  // Subscribers / Publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  // NEW: 로컬 플랜 퍼블리셔
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // 최신 메시지 캐시
  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  nav_msgs::msg::Path latest_path_;

  bool has_odom_{false};
  bool has_scan_{false};
  bool has_path_{false};

  // DWA 파라미터
  double max_vel_x_;
  double min_vel_x_;
  double max_yaw_rate_;
  double acc_x_;
  double acc_yaw_;
  double v_res_;
  double w_res_;
  double dt_;
  double predict_time_;

  double weight_heading_;
  double weight_path_;
  double weight_obstacle_;
  double weight_velocity_;

  double robot_radius_;
  double safety_distance_;
  double wheel_base_;
  double look_ahead_dist_;

  // 마지막 명령 (dynamic window 기준)
  double last_v_{0.0};
  double last_w_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DwaLocalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
