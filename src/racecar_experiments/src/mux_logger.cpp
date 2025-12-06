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
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "planner_mux/msg/mux_status.hpp"

using planner_mux::msg::MuxStatus;
using std::placeholders::_1;
namespace fs = std::filesystem;

class MuxLogger : public rclcpp::Node
{
public:
  enum State { WAITING_FOR_PATH, READY, RUNNING, FINISHED };

  MuxLogger() : Node("mux_logger")
  {
    // --- 파라미터 선언 (두 로거의 파라미터 통합) ---
    output_dir_    = declare_parameter<std::string>("output_dir", "experiment_logs");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "scenario");
    planner_mode_  = declare_parameter<std::string>("planner_mode", "MUX_TEST");
    
    // 도착 판정 관련 (RunLogger 방식: 도로 폭 고려)
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 1.5); 
    start_safe_dist_ = declare_parameter<double>("start_safe_dist", 2.0);

    // Stuck 및 안전 관련
    stuck_timeout_ = declare_parameter<double>("stuck_timeout", 5.0);
    stuck_dist_thresh_ = declare_parameter<double>("stuck_dist_thresh", 0.2);
    safe_dist_threshold_ = declare_parameter<double>("safe_dist_threshold", 0.5);

    // 디렉토리 생성
    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }

    create_log_file();

    // 상태 초기화
    current_state_ = WAITING_FOR_PATH;
    start_time_ = this->now(); 
    last_alive_time_ = start_time_;
    has_left_start_ = false;

    // --- 통계 변수 초기화 ---
    // 1. RunLogger (기본 주행 통계)
    max_speed_ = 0.0; max_a_lat_ = 0.0; total_dist_ = 0.0;
    cnt_frenet_ = 0; cnt_fgm_ = 0; cnt_total_ = 0;
    min_d_min_ = 1e9; unsafe_time_ = 0.0; total_time_ = 0.0;
    sum_track_err_sq_ = 0.0; sum_a_lat_sq_ = 0.0;

    // 2. AvoidLogger (FGM 전용 통계)
    fgm_min_d_min_ = 1e9; 
    fgm_unsafe_time_ = 0.0; 
    fgm_total_time_ = 0.0; 
    fgm_sum_track_err_sq_ = 0.0;
    
    stuck_timer_start_ = this->now();
    
    // --- Subscriber & Publisher ---
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 50, std::bind(&MuxLogger::odom_callback, this, _1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&MuxLogger::path_callback, this, _1));

    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/experiments/crash_detected", 10, std::bind(&MuxLogger::collision_callback, this, _1));

    // [AvoidLogger 기능] 강제 정지를 위한 Publisher
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    mux_sub_ = create_subscription<MuxStatus>(
      "/mux_status", 10,
      std::bind(&MuxLogger::mux_callback, this, _1));
    
    RCLCPP_INFO(get_logger(), "MuxLogger Initialized. Waiting for Path...");
  }

  ~MuxLogger()
  {
    if (current_state_ != FINISHED) {
        finish_logging("MANUAL_ABORT", false); 
    }
  }

private:
  // --- 콜백 함수들 ---

  void mux_callback(const MuxStatus::SharedPtr msg)
  {
    // 현재 선택된 플래너 정보
    last_planner_   = msg->planner;
    last_min_d_     = msg->selected_min_d;
    last_track_err_ = msg->selected_track;

    // [AvoidLogger 기능] 각 플래너의 개별 상태 저장
    last_fre_min_d_ = msg->min_d_frenet;
    last_fgm_min_d_ = msg->min_d_fgm;
    last_fre_track_ = msg->track_frenet;
    last_fgm_track_ = msg->track_fgm;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (current_state_ != WAITING_FOR_PATH) return;
    if (msg->poses.empty()) return;

    // 시작점
    start_x_ = msg->poses.front().pose.position.x;
    start_y_ = msg->poses.front().pose.position.y;
    
    // 목표점 및 방향 (RunLogger 방식: 쿼터니언 -> Yaw 변환)
    goal_x_ = msg->poses.back().pose.position.x;
    goal_y_ = msg->poses.back().pose.position.y;
    double qz = msg->poses.back().pose.orientation.z;
    double qw = msg->poses.back().pose.orientation.w;
    goal_yaw_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

    has_left_start_ = false;
    current_state_ = READY;
    
    RCLCPP_INFO(get_logger(), "Path Received! Waiting for movement...");
  }

  void collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (current_state_ == RUNNING && msg->data == true) {
      is_collision_ = true;
      publish_stop_command(); // [안전] 즉시 정지
      finish_logging("COLLISION DETECTED");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_alive_time_ = this->now();

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

    // 1. 출발 감지
    if (current_state_ == READY) {
        if (speed > 0.1) {
            current_state_ = RUNNING;
            start_time_ = this->now();
            last_pos_x_ = x; last_pos_y_ = y;
            last_odom_time_ = this->now();
            stuck_timer_start_ = this->now();
            stuck_ref_x_ = x; stuck_ref_y_ = y;
            RCLCPP_INFO(get_logger(), "GO! State -> RUNNING");
        }
    }

    // 2. 주행 중 로직
    if (current_state_ == RUNNING) {
        check_stuck(x, y);

        // dt 계산
        rclcpp::Time now = this->now();
        double dt = 0.0;
        if (last_odom_time_.nanoseconds() != 0) {
            dt = (now - last_odom_time_).seconds();
            if (dt < 0.0 || dt > 1.0) dt = 0.0;
        }
        last_odom_time_ = now;

        // --- 데이터 누적 (통합) ---
        total_time_ += dt;
        
        // 전체 안전 통계
        if (last_min_d_ < min_d_min_) min_d_min_ = last_min_d_;
        if (last_min_d_ < safe_dist_threshold_) unsafe_time_ += dt;
        
        // RMS 계산용
        sum_track_err_sq_ += (last_track_err_ * last_track_err_) * dt;
        sum_a_lat_sq_     += (a_lat * a_lat) * dt;

        // [FGM 전용 통계]
        if (last_planner_.find("FGM") != std::string::npos) {
            fgm_total_time_ += dt;
            if (last_fgm_min_d_ < fgm_min_d_min_) fgm_min_d_min_ = last_fgm_min_d_;
            if (last_fgm_min_d_ < safe_dist_threshold_) fgm_unsafe_time_ += dt;
            fgm_sum_track_err_sq_ += (last_fgm_track_ * last_fgm_track_) * dt;
        }

        // 최대값 갱신
        if (speed > max_speed_) max_speed_ = speed;
        if (std::abs(a_lat) > max_a_lat_) max_a_lat_ = std::abs(a_lat);
        total_dist_ += std::hypot(x - last_pos_x_, y - last_pos_y_);
        last_pos_x_ = x; last_pos_y_ = y;

        // 카운트
        cnt_total_++;
        if (last_planner_.find("FRENET") != std::string::npos) cnt_frenet_++;
        else if (last_planner_.find("FGM") != std::string::npos) cnt_fgm_++;

        // --- CSV 로깅 (모든 필드 포함) ---
        if (ofs_.is_open()) {
            double t = (now - start_time_).seconds();
            ofs_ << t << "," 
                 << x << "," << y << "," << yaw << ","
                 << speed << "," << yaw_rate << "," << a_lat << ","
                 << last_planner_ << ","
                 << last_min_d_ << "," << last_track_err_ << "," // 현재 선택된 값
                 << last_fre_min_d_ << "," << last_fgm_min_d_ << "," // 개별 값
                 << last_fre_track_ << "," << last_fgm_track_ << "," // 개별 값
                 << (is_collision_ ? 1 : 0) << "\n";
        }

        // --- 도착 판정 (RunLogger 방식: 로컬 좌표 변환) ---
        if (!has_left_start_) {
            if (std::hypot(x - start_x_, y - start_y_) > start_safe_dist_) has_left_start_ = true;
        }
        
        if (has_left_start_) {
            double dx = x - goal_x_;
            double dy = y - goal_y_;

            // 도착선 기준 앞/뒤 거리 (longitudinal)
            double long_dist = dx * std::cos(goal_yaw_) + dy * std::sin(goal_yaw_);
            // 도착선 기준 좌/우 거리 (lateral)
            double lat_dist = -dx * std::sin(goal_yaw_) + dy * std::cos(goal_yaw_);

            // 조건: 도착선을 지났고(long_dist >= 0) && 트랙 폭 이내(lat_dist < tolerance)
            if (long_dist >= 0.0 && long_dist < 5.0 && std::abs(lat_dist) < goal_tolerance_) {
                publish_stop_command(); // [안전] 정지
                finish_logging("GOAL REACHED");
            }
        }
    }
  }

  void publish_stop_command()
  {
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.header.stamp = this->now();
      drive_msg.header.frame_id = "ego_base_link";
      drive_msg.drive.steering_angle = 0.0;
      drive_msg.drive.speed = 0.0;
      drive_pub_->publish(drive_msg);
  }

  void check_stuck(double curr_x, double curr_y)
  {
      double dist_from_ref = std::hypot(curr_x - stuck_ref_x_, curr_y - stuck_ref_y_);
      if (dist_from_ref > stuck_dist_thresh_) {
          stuck_ref_x_ = curr_x;
          stuck_ref_y_ = curr_y;
          stuck_timer_start_ = this->now();
      } else {
          if ((this->now() - stuck_timer_start_).seconds() > stuck_timeout_) {
              publish_stop_command();
              finish_logging("STUCK DETECTED");
          }
      }
  }

  void create_log_file()
  {
    auto now_t = this->now();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f", now_t.seconds());
    filename_ = output_dir_ + "/" + scenario_name_ + "_" + planner_mode_ + "_" + std::string(buf) + ".csv";

    ofs_.open(filename_);
    if (ofs_.is_open()) {
      // 헤더 작성 (RunLogger + AvoidLogger 모든 필드)
      ofs_ << "t,x,y,yaw,speed,yaw_rate,a_lat,planner,"
           << "min_d,track_error,"
           << "fre_min_d,fgm_min_d,"
           << "fre_track,fgm_track,"
           << "collision\n";
    }
  }

  void finish_logging(const std::string& reason, bool trigger_shutdown = true)
  {
    if (current_state_ == FINISHED) return;
    current_state_ = FINISHED;
    
    publish_stop_command();
    
    if (ofs_.is_open()) ofs_.close();
    
    // --- 통계 계산 ---
    double duration = total_time_;
    
    // 1. Planner 사용 비율
    double frenet_ratio = (cnt_total_ > 0) ? (double)cnt_frenet_ / cnt_total_ * 100.0 : 0.0;
    double fgm_ratio    = (cnt_total_ > 0) ? (double)cnt_fgm_ / cnt_total_ * 100.0 : 0.0;

    // 2. 전체 주행 지표
    double safety_ratio = (total_time_ > 0) ? unsafe_time_ / total_time_ : 0.0;
    double track_rms    = (total_time_ > 0) ? std::sqrt(sum_track_err_sq_ / total_time_) : 0.0;
    double a_lat_rms    = (total_time_ > 0) ? std::sqrt(sum_a_lat_sq_ / total_time_) : 0.0;

    // 3. FGM 전용 지표
    double fgm_safety_ratio = (fgm_total_time_ > 0) ? fgm_unsafe_time_ / fgm_total_time_ : 0.0;
    double fgm_track_rms    = (fgm_total_time_ > 0) ? std::sqrt(fgm_sum_track_err_sq_ / fgm_total_time_) : 0.0;

    // --- CSV 요약 저장 ---
    std::ofstream summary_ofs(filename_, std::ios::app);
    if (summary_ofs.is_open()) {
        summary_ofs << "\n--- Experiment Summary ---\n"
                    << "Scenario," << scenario_name_ << "\n"
                    << "Mode," << planner_mode_ << "\n"
                    << "Finish Reason," << reason << "\n"
                    << "Duration (s)," << std::fixed << std::setprecision(2) << duration << "\n"
                    << "Total Dist (m)," << total_dist_ << "\n"
                    << "Collision," << (is_collision_ ? "YES" : "NO") << "\n"
                    
                    // Planner 비율
                    << "FRENET Usage (%)," << frenet_ratio << "\n"
                    << "FGM Usage (%)," << fgm_ratio << "\n"
                    
                    // 전체 성능
                    << "Max Speed (m/s)," << max_speed_ << "\n"
                    << "Max Lat Acc (m/s^2)," << max_a_lat_ << "\n"
                    << "Global Min Dist (m)," << min_d_min_ << "\n"
                    << "Global Unsafe Ratio," << safety_ratio << "\n"
                    << "Global Track RMS (m)," << track_rms << "\n"
                    << "Global Lat Acc RMS," << a_lat_rms << "\n"
                    
                    // FGM 성능
                    << "FGM Min Dist (m)," << fgm_min_d_min_ << "\n"
                    << "FGM Unsafe Ratio," << fgm_safety_ratio << "\n"
                    << "FGM Track RMS (m)," << fgm_track_rms << "\n";
        summary_ofs.close();
    }

    // 파일명 변경
    try {
        fs::path old_path(filename_);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << duration;
        std::string tag = "_ABORT";
        if (reason.find("GOAL") != std::string::npos) tag = "_GOAL";
        else if (reason.find("COLLISION") != std::string::npos) tag = "_CRASH";
        else if (reason.find("STUCK") != std::string::npos) tag = "_STUCK";

        std::string new_name = old_path.stem().string() + tag + "_dur_" + ss.str() + "s" + old_path.extension().string();
        fs::rename(old_path, old_path.parent_path() / new_name);
        filename_ = (old_path.parent_path() / new_name).string();
    } catch (...) {}

    // --- 콘솔 출력 (통합) ---
    std::cout << "\n========================================\n";
    std::cout << " [MuxLogger] Experiment Result Summary \n";
    std::cout << "========================================\n";
    std::cout << " Finish Reason : " << reason << "\n";
    std::cout << " Duration      : " << duration << " s\n";
    std::cout << " Total Dist    : " << total_dist_ << " m\n";
    std::cout << " Collision     : " << (is_collision_ ? "YES" : "NO") << "\n";
    std::cout << "----------------------------------------\n";
    std::cout << " [Planner Usage]\n";
    std::cout << " FRENET        : " << frenet_ratio << " %\n";
    std::cout << " FGM           : " << fgm_ratio << " %\n";
    std::cout << "----------------------------------------\n";
    std::cout << " [Global Performance]\n";
    std::cout << " Max Speed     : " << max_speed_ << " m/s\n";
    std::cout << " Track RMS     : " << track_rms << " m\n";
    std::cout << " Min Dist      : " << min_d_min_ << " m (Unsafe: " << safety_ratio*100 << "%)\n";
    std::cout << "----------------------------------------\n";
    std::cout << " [FGM Specific Performance]\n";
    std::cout << " FGM Min Dist  : " << fgm_min_d_min_ << " m\n";
    std::cout << " FGM Unsafe %  : " << fgm_safety_ratio * 100.0 << " %\n";
    std::cout << " FGM Track RMS : " << fgm_track_rms << " m\n";
    std::cout << "========================================\n";
    std::cout << " Log Saved to  : " << filename_ << "\n\n";

    if (trigger_shutdown && rclcpp::ok()) {
        rclcpp::shutdown();
    }
  }

  // --- Members ---
  std::string output_dir_, scenario_name_, planner_mode_;
  double goal_tolerance_, start_safe_dist_;
  double stuck_timeout_, stuck_dist_thresh_;
  double safe_dist_threshold_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<MuxStatus>::SharedPtr mux_sub_;

  State current_state_;
  rclcpp::Time start_time_, last_alive_time_, last_odom_time_, stuck_timer_start_;
  
  std::string filename_;
  std::ofstream ofs_;

  double start_x_, start_y_, goal_x_, goal_y_, goal_yaw_;
  double stuck_ref_x_, stuck_ref_y_;
  
  bool has_left_start_ = false; 
  bool is_collision_ = false;

  double max_speed_, max_a_lat_, total_dist_;
  double last_pos_x_, last_pos_y_;
  long cnt_frenet_, cnt_fgm_, cnt_total_;

  std::string last_planner_ {"NONE"};
  double last_min_d_, last_track_err_;
  double last_fre_min_d_, last_fgm_min_d_, last_fre_track_, last_fgm_track_;

  // Global 통계
  double min_d_min_, unsafe_time_, total_time_, sum_track_err_sq_, sum_a_lat_sq_;
  
  // FGM 전용 통계
  double fgm_min_d_min_, fgm_unsafe_time_, fgm_total_time_, fgm_sum_track_err_sq_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MuxLogger>());
  rclcpp::shutdown();
  return 0;
}