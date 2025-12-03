#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <iomanip>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "planner_mux_msgs/msg/mux_debug.hpp"

using std::placeholders::_1;
using planner_mux_msgs::msg::MuxDebug;
namespace fs = std::filesystem;

class RunLogger : public rclcpp::Node
{
public:
  enum State { WAITING_FOR_PATH, READY, RUNNING, FINISHED };

  RunLogger() : Node("run_logger")
  {
    // --- 기본 파라미터 ---
    output_dir_    = declare_parameter<std::string>("output_dir", "experiment_logs");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "scenario");
    planner_mode_  = declare_parameter<std::string>("planner_mode", "MUX");
    
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.5); 
    start_safe_dist_ = declare_parameter<double>("start_safe_dist", 2.0);

    stuck_timeout_ = declare_parameter<double>("stuck_timeout", 5.0);
    stuck_dist_thresh_ = declare_parameter<double>("stuck_dist_thresh", 0.2);

    // MUX 가중치
    w_speed_ = declare_parameter<double>("w_speed", 0.0);
    w_track_ = declare_parameter<double>("w_track", 0.0);
    w_comfort_ = declare_parameter<double>("w_comfort", 0.0);
    w_safety_ = declare_parameter<double>("w_safety", 0.0);

    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }

    create_log_file();

    current_state_ = WAITING_FOR_PATH;
    start_time_ = rclcpp::Time(0);
    has_left_start_ = false;

    max_speed_ = 0.0;
    max_a_lat_ = 0.0;
    total_dist_ = 0.0;
    
    cnt_frenet_ = 0;
    cnt_fgm_ = 0;
    cnt_total_ = 0;
    
    stuck_timer_start_ = rclcpp::Time(0);
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 50, std::bind(&RunLogger::odom_callback, this, _1));

    debug_sub_ = create_subscription<MuxDebug>(
      "/mux_debug", 50, std::bind(&RunLogger::debug_callback, this, _1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&RunLogger::path_callback, this, _1));

    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/experiments/crash_detected", 10, std::bind(&RunLogger::collision_callback, this, _1));

    RCLCPP_INFO(get_logger(), "RunLogger initialized. Stuck timeout: %.1fs", stuck_timeout_);
  }

  // [수정됨] 소멸자에서 강제 종료 처리
  ~RunLogger()
  {
    // 만약 프로그램이 종료되는데(Ctrl+C), 상태가 아직 FINISHED가 아니라면
    // 강제로 로그를 마무리하고 요약본을 출력합니다.
    if (current_state_ == RUNNING || current_state_ == READY) {
        // 소멸자 내부에서는 ROS Time을 쓰기 어려울 수 있으므로 예외처리
        try {
            finish_logging("MANUAL_ABORT (Ctrl+C)", false); // false = shutdown 호출 안 함
        } catch (...) {
            std::cout << "[RunLogger] Saved log during shutdown.\n";
        }
    }

    if (ofs_.is_open()) ofs_.close();
  }

private:
  // --- 콜백 ---

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (current_state_ != WAITING_FOR_PATH) return;
    if (msg->poses.empty()) return;

    start_x_ = msg->poses.front().pose.position.x;
    start_y_ = msg->poses.front().pose.position.y;
    goal_x_ = msg->poses.back().pose.position.x;
    goal_y_ = msg->poses.back().pose.position.y;

    has_left_start_ = false;
    current_state_ = READY;
    
    RCLCPP_INFO(get_logger(), "Path received! Waiting for movement...");
  }

  void collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (current_state_ == RUNNING && msg->data == true) {
      is_collision_ = true;
      finish_logging("COLLISION DETECTED");
    }
  }

  void debug_callback(const MuxDebug::SharedPtr msg)
  {
    last_planner_ = msg->current_planner;
    last_min_d_ = std::min(msg->fre_min_d, msg->fgm_min_d);
    last_track_err_ = std::min(msg->fre_track, msg->fgm_track);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double speed = std::sqrt(vx * vx + vy * vy);

    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
    double yaw_rate = msg->twist.twist.angular.z;
    double a_lat = speed * yaw_rate;

    if (current_state_ == READY) {
        if (speed > 0.1) {
            current_state_ = RUNNING;
            start_time_ = this->now();
            last_pos_x_ = x;
            last_pos_y_ = y;
            stuck_timer_start_ = this->now();
            stuck_ref_x_ = x;
            stuck_ref_y_ = y;
            RCLCPP_INFO(get_logger(), "Movement detected! State -> RUNNING.");
        }
    }

    if (current_state_ == RUNNING) {
        check_stuck(x, y);

        if (speed > max_speed_) max_speed_ = speed;
        if (std::abs(a_lat) > max_a_lat_) max_a_lat_ = std::abs(a_lat);
        
        double step_dist = std::hypot(x - last_pos_x_, y - last_pos_y_);
        total_dist_ += step_dist;
        last_pos_x_ = x;
        last_pos_y_ = y;

        cnt_total_++;
        if (last_planner_.find("FRENET") != std::string::npos) cnt_frenet_++;
        else if (last_planner_.find("FGM") != std::string::npos) cnt_fgm_++;

        if (ofs_.is_open()) {
            double t = 0.0;
            try {
                t = (this->now() - start_time_).seconds();
            } catch(...) {}
            
            ofs_ << t << ","
                 << x << "," << y << "," << yaw << ","
                 << speed << "," << yaw_rate << "," << a_lat << ","
                 << last_planner_ << "," << last_min_d_ << "," << last_track_err_ << ","
                 << (is_collision_ ? 1 : 0) << "\n";
        }

        if (!has_left_start_) {
            if (std::hypot(x - start_x_, y - start_y_) > start_safe_dist_) {
                has_left_start_ = true;
            }
        }
        if (has_left_start_) {
            if (std::hypot(x - goal_x_, y - goal_y_) < goal_tolerance_) {
                finish_logging("GOAL REACHED");
            }
        }
    }
  }

  void check_stuck(double curr_x, double curr_y)
  {
      double dist_from_ref = std::hypot(curr_x - stuck_ref_x_, curr_y - stuck_ref_y_);
      if (dist_from_ref > stuck_dist_thresh_) {
          stuck_ref_x_ = curr_x;
          stuck_ref_y_ = curr_y;
          stuck_timer_start_ = this->now();
      } else {
          double elapsed_stuck = (this->now() - stuck_timer_start_).seconds();
          if (elapsed_stuck > stuck_timeout_) {
              finish_logging("STUCK DETECTED");
          }
      }
  }

  void create_log_file()
  {
    auto now_t = this->now();
    auto sec   = now_t.seconds();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f", sec);
    std::string ts(buf);

    filename_ = output_dir_ + "/" + scenario_name_ + "_" +
                planner_mode_ + "_" + ts + ".csv";

    ofs_.open(filename_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", filename_.c_str());
    } else {
      ofs_ << "t,x,y,yaw,speed,yaw_rate,a_lat,planner,min_d,track_error,collision\n";
    }
  }

  // [수정됨] trigger_shutdown 파라미터 추가
  // Ctrl+C로 종료될 때는 이미 셧다운 중이므로 rclcpp::shutdown()을 부르면 안 됨.
  void finish_logging(const std::string& reason, bool trigger_shutdown = true)
  {
    if (current_state_ == FINISHED) return;
    current_state_ = FINISHED;
    
    if (ofs_.is_open()) ofs_.close();
    
    double duration = 0.0;
    // rclcpp context가 살아있을 때만 시간 계산
    if (rclcpp::ok()) {
        try {
            duration = (this->now() - start_time_).seconds();
        } catch (...) {
            duration = 0.0;
        }
    }

    try {
        fs::path old_path(filename_);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << duration;
        std::string new_filename = old_path.stem().string() + "_dur_" + ss.str() + "s" + old_path.extension().string();
        fs::path new_path = old_path.parent_path() / new_filename;

        fs::rename(old_path, new_path);
        filename_ = new_path.string();
    } catch (const std::exception& e) {
        std::cout << "[RunLogger] Error renaming file: " << e.what() << "\n";
    }

    print_summary(reason, duration);
    
    // 내부 로직(충돌, 목표 도달)에 의한 종료일 때만 시스템 셧다운 트리거
    if (trigger_shutdown && rclcpp::ok()) {
        rclcpp::shutdown();
    }
  }

  void print_summary(const std::string& reason, double duration)
  {
    double frenet_ratio = (cnt_total_ > 0) ? (double)cnt_frenet_ / cnt_total_ * 100.0 : 0.0;
    double fgm_ratio    = (cnt_total_ > 0) ? (double)cnt_fgm_ / cnt_total_ * 100.0 : 0.0;

    // Ctrl+C 종료 시 ROS 로그가 안 보일 수 있으므로 std::cout 사용
    std::cout << "\n========================================\n";
    std::cout << " [RunLogger] Experiment Result Summary \n";
    std::cout << "========================================\n";
    std::cout << " Scenario      : " << scenario_name_ << " (Mode: " << planner_mode_ << ")\n";
    std::cout << " Finish Reason : " << reason << "\n";
    std::cout << " Duration      : " << std::fixed << std::setprecision(2) << duration << " s\n";
    std::cout << " Total Dist    : " << total_dist_ << " m\n";
    std::cout << " Max Speed     : " << max_speed_ << " m/s\n";
    std::cout << " Max Lat Acc   : " << max_a_lat_ << " m/s^2\n";
    std::cout << " Collision     : " << (is_collision_ ? "YES" : "NO") << "\n";
    std::cout << "----------------------------------------\n";
    std::cout << " [Planner Usage Ratio]\n";
    std::cout << " FRENET        : " << frenet_ratio << " %\n";
    std::cout << " FGM           : " << fgm_ratio << " %\n";
    std::cout << "----------------------------------------\n";
    std::cout << " [MUX Weights Used]\n";
    std::cout << " w_speed       : " << w_speed_ << "\n";
    std::cout << " w_track       : " << w_track_ << "\n";
    std::cout << " w_comfort     : " << w_comfort_ << "\n";
    std::cout << " w_safety      : " << w_safety_ << "\n";
    std::cout << "========================================\n";
    std::cout << " Log Saved to  : " << filename_ << "\n";
    std::cout << "========================================\n\n";
  }

  // --- Members ---
  std::string output_dir_, scenario_name_, planner_mode_;
  double goal_tolerance_, start_safe_dist_;
  double w_speed_, w_track_, w_comfort_, w_safety_;
  double stuck_timeout_, stuck_dist_thresh_;
  rclcpp::Time stuck_timer_start_;
  double stuck_ref_x_, stuck_ref_y_;

  std::string filename_;
  std::ofstream ofs_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<MuxDebug>::SharedPtr debug_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;

  State current_state_;
  rclcpp::Time start_time_;
  
  double start_x_, start_y_, goal_x_, goal_y_;
  bool has_left_start_ = false; 
  bool is_collision_ = false;

  double max_speed_, max_a_lat_, total_dist_;
  double last_pos_x_, last_pos_y_;
  long cnt_frenet_, cnt_fgm_, cnt_total_;

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