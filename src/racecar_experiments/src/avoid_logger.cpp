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
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // [필수] 정지 명령용

using std::placeholders::_1;
namespace fs = std::filesystem;

class AvoidLogger : public rclcpp::Node
{
public:
  enum State { WAITING_FOR_PATH, READY, RUNNING, FINISHED };

  AvoidLogger() : Node("avoid_logger")
  {
    // --- 파라미터 설정 ---
    output_dir_    = declare_parameter<std::string>("output_dir", "experiment_logs");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "scenario");
    planner_mode_  = declare_parameter<std::string>("planner_mode", "FGM_TEST");
    
    // [중요] 목표 반경(Tolerance)은 보조 수단이고, 주 수단은 '결승선 통과'임
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 2.0); 
    start_safe_dist_ = declare_parameter<double>("start_safe_dist", 2.0);

    // Stuck 감지 (어딘가에 끼어서 못 움직일 때 종료)
    stuck_timeout_ = declare_parameter<double>("stuck_timeout", 5.0);
    stuck_dist_thresh_ = declare_parameter<double>("stuck_dist_thresh", 0.2);
    
    // 안전 거리 임계값 (Unsafe Time 계산용)
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

    // 통계 변수 초기화
    max_speed_ = 0.0; max_a_lat_ = 0.0; total_dist_ = 0.0;
    cnt_frenet_ = 0; cnt_fgm_ = 0; cnt_total_ = 0;
    
    // 전체(MUX) 통계
    min_d_min_ = 1e9; unsafe_time_ = 0.0; total_time_ = 0.0;
    sum_track_err_sq_ = 0.0; sum_a_lat_sq_ = 0.0;

    // [핵심] FGM 전용 통계
    fgm_min_d_min_ = 1e9; 
    fgm_unsafe_time_ = 0.0; 
    fgm_total_time_ = 0.0; 
    fgm_sum_track_err_sq_ = 0.0;
    
    stuck_timer_start_ = this->now();
    
    // --- Subscriber & Publisher ---
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 50, std::bind(&AvoidLogger::odom_callback, this, _1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&AvoidLogger::path_callback, this, _1));

    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/experiments/crash_detected", 10, std::bind(&AvoidLogger::collision_callback, this, _1));

    // [추가] 강제 정지를 위한 Publisher
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    RCLCPP_INFO(get_logger(), "AvoidLogger Initialized. Waiting for Path to define Finish Line...");
  }

  ~AvoidLogger()
  {
    if (current_state_ != FINISHED) {
        finish_logging("MANUAL_ABORT", false); 
    }
  }

private:
  // --- Callback Functions ---

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (current_state_ != WAITING_FOR_PATH) return;
    if (msg->poses.size() < 2) return;

    // 시작점 설정
    start_x_ = msg->poses.front().pose.position.x;
    start_y_ = msg->poses.front().pose.position.y;
    
    // [핵심 로직] 결승선 벡터 계산 (마지막 두 점 이용)
    size_t last_idx = msg->poses.size() - 1;
    goal_x_ = msg->poses[last_idx].pose.position.x;
    goal_y_ = msg->poses[last_idx].pose.position.y;

    double prev_x = msg->poses[last_idx - 1].pose.position.x;
    double prev_y = msg->poses[last_idx - 1].pose.position.y;

    // 경로의 마지막 진행 방향 벡터 (Finish Line Normal Vector)
    finish_line_vec_x_ = goal_x_ - prev_x;
    finish_line_vec_y_ = goal_y_ - prev_y;

    // 벡터 정규화 (Unit Vector)
    double norm = std::hypot(finish_line_vec_x_, finish_line_vec_y_);
    if (norm > 1e-6) {
        finish_line_vec_x_ /= norm;
        finish_line_vec_y_ /= norm;
    }

    has_left_start_ = false;
    current_state_ = READY;
    
    RCLCPP_INFO(get_logger(), "Path Received! Finish Line Vector: (%.3f, %.3f)", 
                finish_line_vec_x_, finish_line_vec_y_);
  }

  void collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (current_state_ == RUNNING && msg->data == true) {
      is_collision_ = true;
      publish_stop_command(); // [즉시 정지]
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

        // --- 데이터 누적 ---
        total_time_ += dt;
        if (last_min_d_ < min_d_min_) min_d_min_ = last_min_d_;
        if (last_min_d_ < safe_dist_threshold_) unsafe_time_ += dt;
        sum_track_err_sq_ += (last_track_err_ * last_track_err_) * dt;
        sum_a_lat_sq_     += (a_lat * a_lat) * dt;

        // [중요] FGM 활성 구간 전용 통계
        if (last_planner_.find("FGM") != std::string::npos) {
            fgm_total_time_ += dt;
            if (last_fgm_min_d_ < fgm_min_d_min_) fgm_min_d_min_ = last_fgm_min_d_;
            if (last_fgm_min_d_ < safe_dist_threshold_) fgm_unsafe_time_ += dt;
            fgm_sum_track_err_sq_ += (last_fgm_track_ * last_fgm_track_) * dt;
        }

        // 기타 통계
        if (speed > max_speed_) max_speed_ = speed;
        if (std::abs(a_lat) > max_a_lat_) max_a_lat_ = std::abs(a_lat);
        total_dist_ += std::hypot(x - last_pos_x_, y - last_pos_y_);
        last_pos_x_ = x; last_pos_y_ = y;

        cnt_total_++;
        if (last_planner_.find("FRENET") != std::string::npos) cnt_frenet_++;
        else if (last_planner_.find("FGM") != std::string::npos) cnt_fgm_++;

        // --- CSV 로깅 (Raw Data) ---
        if (ofs_.is_open()) {
            double t = (now - start_time_).seconds();
            ofs_ << t << "," 
                 << x << "," << y << "," << yaw << ","
                 << speed << "," << yaw_rate << "," << a_lat << ","
                 << last_planner_ << ","
                 << last_min_d_ << "," << last_track_err_ << "," // MUX 선택값
                 << last_fre_min_d_ << "," << last_fgm_min_d_ << "," // 개별 값
                 << last_fre_track_ << "," << last_fgm_track_ << "," // 개별 값
                 << (is_collision_ ? 1 : 0) << "\n";
        }

        // --- [핵심] 결승선 통과 체크 ---
        if (!has_left_start_) {
            if (std::hypot(x - start_x_, y - start_y_) > start_safe_dist_) has_left_start_ = true;
        }
        
        if (has_left_start_) {
            double dist_to_goal = std::hypot(x - goal_x_, y - goal_y_);
            
            // 내적 계산: (차량위치 - 목표위치) • (목표진행방향)
            // 값이 양수(+)면 목표 지점을 지나쳤다는 의미
            double vec_x = x - goal_x_;
            double vec_y = y - goal_y_;
            double dot_prod = vec_x * finish_line_vec_x_ + vec_y * finish_line_vec_y_;

            // 조건: 목표 근처(5m)에 도달했고 && 결승선을 넘었으면 종료
            bool crossed_finish_line = (dist_to_goal < 5.0 && dot_prod > 0.0);
            
            if (crossed_finish_line || dist_to_goal < goal_tolerance_) {
                publish_stop_command(); // [즉시 정지]
                finish_logging("GOAL REACHED");
            }
        }
    }
  }

  // 강제 정지 명령 발행 함수
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
      // 헤더 작성 (FGM 전용 컬럼 포함)
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
    
    // 종료 시 한번 더 정지 명령
    publish_stop_command();
    
    if (ofs_.is_open()) ofs_.close();
    
    double duration = total_time_;
    
    // 통계 계산
    double safety_ratio = (total_time_ > 0) ? unsafe_time_ / total_time_ : 0.0;
    
    // [중요] FGM 전용 지표 계산
    double fgm_safety_ratio = (fgm_total_time_ > 0) ? fgm_unsafe_time_ / fgm_total_time_ : 0.0;
    double fgm_track_rms = (fgm_total_time_ > 0) ? std::sqrt(fgm_sum_track_err_sq_ / fgm_total_time_) : 0.0;

    // 요약 파일 저장 (Append Mode)
    std::ofstream summary_ofs(filename_, std::ios::app);
    if (summary_ofs.is_open()) {
        summary_ofs << "\n--- Experiment Summary ---\n"
                    << "Scenario," << scenario_name_ << "\n"
                    << "Finish Reason," << reason << "\n"
                    << "Duration (s)," << std::fixed << std::setprecision(2) << duration << "\n"
                    << "Total Dist (m)," << total_dist_ << "\n"
                    << "Collision," << (is_collision_ ? "YES" : "NO") << "\n"
                    << "Min Dist (m)," << min_d_min_ << "\n"
                    << "Unsafe Ratio," << safety_ratio << "\n"
                    // FGM 전용 지표 기록
                    << "FGM Min Dist (m)," << fgm_min_d_min_ << "\n"
                    << "FGM Unsafe Ratio," << fgm_safety_ratio << "\n"
                    << "FGM Track RMS (m)," << fgm_track_rms << "\n";
        summary_ofs.close();
    }

    // 파일명 변경 (결과 태그 추가)
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

    // 콘솔 출력
    std::cout << "\n[AvoidLogger] Finished: " << reason << "\n";
    std::cout << " - Duration: " << duration << "s\n";
    std::cout << " - FGM Min Dist: " << fgm_min_d_min_ << "m\n";
    std::cout << " - FGM Unsafe Ratio: " << fgm_safety_ratio << "\n";
    std::cout << " - Log: " << filename_ << "\n\n";

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

  State current_state_;
  rclcpp::Time start_time_, last_alive_time_, last_odom_time_, stuck_timer_start_;
  
  std::string filename_;
  std::ofstream ofs_;

  double start_x_, start_y_, goal_x_, goal_y_;
  double finish_line_vec_x_ = 0.0, finish_line_vec_y_ = 0.0;
  
  bool has_left_start_ = false; 
  bool is_collision_ = false;

  double max_speed_, max_a_lat_, total_dist_;
  double last_pos_x_, last_pos_y_, stuck_ref_x_, stuck_ref_y_;
  long cnt_frenet_, cnt_fgm_, cnt_total_;

  std::string last_planner_ {"NONE"};
  double last_min_d_, last_track_err_;
  double last_fre_min_d_, last_fgm_min_d_, last_fre_track_, last_fgm_track_;

  double min_d_min_, unsafe_time_, total_time_, sum_track_err_sq_, sum_a_lat_sq_;
  
  // FGM 전용 멤버
  double fgm_min_d_min_, fgm_unsafe_time_, fgm_total_time_, fgm_sum_track_err_sq_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidLogger>());
  rclcpp::shutdown();
  return 0;
}