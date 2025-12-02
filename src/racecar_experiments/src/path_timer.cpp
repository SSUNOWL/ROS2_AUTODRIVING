#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"

class PathTimer : public rclcpp::Node
{
public:
  PathTimer() : Node("path_timer"), state_(IDLE), has_path_(false), has_left_start_(false)
  {
    // 파라미터: 도착 판정 거리 (기본 0.5m)
    this->declare_parameter("goal_tolerance", 0.5);

    // 1. 전역 경로 구독 (QoS 10: Reliable)
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&PathTimer::path_callback, this, std::placeholders::_1));

    // 2. 오도메트리(내 위치) 구독
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&PathTimer::odom_callback, this, std::placeholders::_1));

    // 3. 결과 시간 발행
    pub_time_ = this->create_publisher<std_msgs::msg::Float32>("/experiments/travel_time", 10);

    RCLCPP_INFO(this->get_logger(), "Path Timer Ready. Waiting for Global Plan...");
  }

private:
  enum State { IDLE, RUNNING, FINISHED };
  State state_;
  bool has_path_;
  bool has_left_start_; // 출발지에서 벗어났는지 확인하는 플래그

  // 저장된 시작점과 도착점
  double start_x_, start_y_;
  double goal_x_, goal_y_;
  
  rclcpp::Time start_time_;

  // [핵심] 전역 경로가 들어오면 시작점/끝점을 추출
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path!");
        return;
    }

    // 디버깅용 로그 추가
    // RCLCPP_INFO(this->get_logger(), "Received Path with %zu points.", msg->poses.size());

    double sx = msg->poses.front().pose.position.x;
    double sy = msg->poses.front().pose.position.y;
    double gx = msg->poses.back().pose.position.x;
    double gy = msg->poses.back().pose.position.y;

    // [수정됨] 폐곡선 트랙(Start==Goal)인 경우 단순 거리 계산은 0이 되므로
    // 시작점-도착점 거리 대신 '점의 개수'로 유효성을 판단하거나, 이 검사를 제거해야 합니다.
    if (msg->poses.size() < 10) 
    {
       RCLCPP_WARN(this->get_logger(), "Path is too short (points < 10). Ignoring.");
       return; 
    }
    
    // 이미 달리고 있는 중이라면(RUNNING), 시작점은 바꾸지 않고 도착점만 업데이트
    if (state_ == IDLE || state_ == FINISHED)
    {
      start_x_ = sx;
      start_y_ = sy;
      state_ = IDLE;
      has_left_start_ = false; // 플래그 초기화
      RCLCPP_INFO(this->get_logger(), "New Path Set! Start: (%.2f, %.2f) -> Goal: (%.2f, %.2f)", 
        start_x_, start_y_, gx, gy);
    }

    goal_x_ = gx;
    goal_y_ = gy;
    
    has_path_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_path_) return;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double speed = msg->twist.twist.linear.x; // 로봇의 전진 속도
    double tolerance = this->get_parameter("goal_tolerance").as_double();

    // 현재 위치와 시작점/도착점 사이 거리 계산
    double dist_to_start = std::sqrt(std::pow(x - start_x_, 2) + std::pow(y - start_y_, 2));
    double dist_to_goal = std::sqrt(std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));

    switch (state_)
    {
      case IDLE:
        // 시작점 근처(1.5m 이내)에 있고 + 속도가 0.1 m/s 이상 붙으면 측정 시작
        // (단, 텔레포트 직후 속도가 튀는 것을 방지하기 위해 너무 먼 거리면 시작 안 함)
        if (dist_to_start < 2.0 && std::abs(speed) > 0.1)
        {
          start_time_ = this->get_clock()->now();
          state_ = RUNNING;
          has_left_start_ = false; 
          RCLCPP_INFO(this->get_logger(), "Timer Started! GO GO GO!");
        }
        break;

      case RUNNING:
      {
        // [중요] 루프 트랙의 경우 시작하자마자 도착점으로 인식되는 문제 방지
        // 시작점에서 최소 3m 이상 멀어진 적이 있어야 도착 판정을 시작함
        if (!has_left_start_) {
            if (dist_to_start > 3.0) {
                has_left_start_ = true;
                RCLCPP_INFO(this->get_logger(), "Vehicle left start zone. Goal check enabled.");
            }
        }

        // 도착점 근처에 도달했는지 확인 (출발지를 벗어난 이후에만 체크)
        if (has_left_start_ && dist_to_goal < tolerance)
        {
          double elapsed = (this->get_clock()->now() - start_time_).seconds();
          state_ = FINISHED;

          RCLCPP_INFO(this->get_logger(), "GOAL REACHED! Time: %.4f seconds", elapsed);
          
          auto time_msg = std_msgs::msg::Float32();
          time_msg.data = elapsed;
          pub_time_->publish(time_msg);
        }
        break;
      }

      case FINISHED:
        // 차량이 멈추고 다시 시작점 근처로 오면 IDLE로 리셋 (다음 실험 준비)
        if (dist_to_start < 1.0 && std::abs(speed) < 0.05)
        {
             state_ = IDLE;
             has_left_start_ = false;
             RCLCPP_INFO(this->get_logger(), "Resetting Timer for next run...");
        }
        break;
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTimer>());
  rclcpp::shutdown();
  return 0;
}