#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "planner_mux_msgs/msg/mux_debug.hpp"

using std::placeholders::_1;
using planner_mux_msgs::msg::MuxDebug;

class RunLogger : public rclcpp::Node
{
public:
  RunLogger() : Node("run_logger")
  {
    // 파라미터
    output_dir_    = declare_parameter<std::string>("output_dir", "experiment_logs");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "scenario");
    planner_mode_  = declare_parameter<std::string>("planner_mode", "MUX");

    // 파일 이름: scenario_planner_timestamp.csv
    auto now_t = this->now();
    auto sec   = now_t.seconds();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f", sec);
    std::string ts(buf);

    filename_ = output_dir_ + "/" + scenario_name_ + "_" +
                planner_mode_ + "_" + ts + ".csv";

    // 디렉토리 없으면 mkdir은 여기선 생략(이미 있을 거라고 가정)
    ofs_.open(filename_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", filename_.c_str());
    } else {
      // CSV 헤더
      ofs_ << "t,x,y,yaw,speed,yaw_rate,a_lat,"
              "planner,min_d,track_error\n";
    }

    start_time_ = this->now();

    // /odom 구독
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 50,
      std::bind(&RunLogger::odom_callback, this, _1));

    // /mux_debug 구독 (planner, min_d, track_error 등)
    debug_sub_ = create_subscription<MuxDebug>(
      "/mux_debug", 50,
      std::bind(&RunLogger::debug_callback, this, _1));

    RCLCPP_INFO(get_logger(), "RunLogger started, logging to %s", filename_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!ofs_.is_open()) {
      return;
    }

    // 경과 시간
    double t = (this->now() - start_time_).seconds();

    // pose → yaw 추출
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

    // 속도 / yaw_rate / 횡가속
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double speed = std::sqrt(vx * vx + vy * vy);

    double yaw_rate = msg->twist.twist.angular.z;
    double a_lat = speed * yaw_rate;

    // MUX 디버그에서 마지막으로 받은 값
    std::string planner = last_planner_;
    double min_d = last_min_d_;
    double track_error = last_track_err_;

    ofs_ << t << ","
         << x << "," << y << "," << yaw << ","
         << speed << "," << yaw_rate << "," << a_lat << ","
         << planner << "," << min_d << "," << track_error
         << "\n";
  }

  void debug_callback(const MuxDebug::SharedPtr msg)
  {
    // 현재 선택된 플래너 이름
    last_planner_ = msg->current_planner;

    // 두 플래너 중 더 위험한(거리 짧은) 쪽을 min_d로 사용
    double d_fre = msg->fre_min_d;
    double d_fgm = msg->fgm_min_d;
    last_min_d_ = std::min(d_fre, d_fgm);

    // track error도 두 플래너 중 더 나쁜 쪽 or 더 좋은 쪽을 택해도 되는데,
    // 여기선 "더 좋은 쪽"(작은 쪽)을 기록
    last_track_err_ = std::min(msg->fre_track, msg->fgm_track);
  }

  // 멤버
  std::string output_dir_;
  std::string scenario_name_;
  std::string planner_mode_;

  std::string filename_;
  std::ofstream ofs_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MuxDebug>::SharedPtr debug_sub_;

  rclcpp::Time start_time_;

  std::string last_planner_ {"NONE"};
  double last_min_d_ {999.9};
  double last_track_err_ {0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RunLogger>());
  rclcpp::shutdown();
  return 0;
}
