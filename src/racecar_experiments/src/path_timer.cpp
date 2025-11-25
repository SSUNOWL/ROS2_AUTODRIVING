#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"

class PathTimer : public rclcpp::Node
{
public:
  PathTimer() : Node("path_timer"), state_(IDLE), has_path_(false)
  {
    // 파라미터: 도착 판정 거리 (기본 0.5m)
    this->declare_parameter("goal_tolerance", 0.5);

    // 1. 전역 경로 구독 (Nav2는 '/plan', 다른 알고리즘도 이 토픽을 쓰거나 리매핑하면 됨)
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&PathTimer::path_callback, this, std::placeholders::_1));

    // 2. 오도메트리(내 위치) 구독
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&PathTimer::odom_callback, this, std::placeholders::_1));

    // 3. 결과 시간 발행
    pub_time_ = this->create_publisher<std_msgs::msg::Float32>("/experiments/travel_time", 10);

    RCLCPP_INFO(this->get_logger(), "⏱️ Path Timer Ready. Waiting for Global Plan...");
  }

private:
  enum State { IDLE, RUNNING, FINISHED };
  State state_;
  bool has_path_;

  // 저장된 시작점과 도착점
  double start_x_, start_y_;
  double goal_x_, goal_y_;
  
  rclcpp::Time start_time_;

  // [핵심] 전역 경로가 들어오면 시작점/끝점을 추출
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;
    double sx = msg->poses.front().pose.position.x;
    double sy = msg->poses.front().pose.position.y;
    double gx = msg->poses.back().pose.position.x;
    double gy = msg->poses.back().pose.position.y;

    // [추가] 시작점과 목표점 사이 거리가 너무 짧으면 무시 (2m 이하)
    double path_len = std::sqrt(std::pow(sx - gx, 2) + std::pow(sy - gy, 2));
    if (path_len < 2.0) 
    {
        // 너무 짧은 경로(제자리 경로)는 무시
        return; 
    }

    
    // 이미 달리고 있는 중이라면(RUNNING), 시작점은 바꾸지 않고 도착점만 업데이트합니다.
    // (이유: A*는 로봇이 움직이면 시작점을 계속 현재 위치로 바꾸기 때문에, 처음 출발 위치를 기억해야 함)
    if (state_ == IDLE || state_ == FINISHED)
    {
      // 시작점 저장 (경로의 0번째 점)
      start_x_ = msg->poses.front().pose.position.x;
      start_y_ = msg->poses.front().pose.position.y;
      state_ = IDLE; // 새로운 경로가 오면 상태 리셋
      RCLCPP_INFO(this->get_logger(), "New Path Received! Start: (%.1f, %.1f)", start_x_, start_y_);
    }

    // 도착점 저장 (경로의 마지막 점)
    goal_x_ = msg->poses.back().pose.position.x;
    goal_y_ = msg->poses.back().pose.position.y;
    
    has_path_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_path_) return;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double speed = msg->twist.twist.linear.x;
    double tolerance = this->get_parameter("goal_tolerance").as_double();

    // 현재 위치와 시작점/도착점 사이 거리 계산
    double dist_to_start = std::sqrt(std::pow(x - start_x_, 2) + std::pow(y - start_y_, 2));
    double dist_to_goal = std::sqrt(std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));

    switch (state_)
    {
      case IDLE:
        // 시작점 근처(1.0m)에 있고 + 속도가 붙으면 측정 시작
        if (dist_to_start < 1.0 && std::abs(speed) > 0.1)
        {
          start_time_ = this->get_clock()->now();
          state_ = RUNNING;
          RCLCPP_INFO(this->get_logger(), "Timer Started! Head to Goal (%.1f, %.1f)", goal_x_, goal_y_);
        }
        break;

      case RUNNING:
      {
        // 도착점 근처에 도달했는지 확인
        if (dist_to_goal < tolerance)
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
        // 도착 후 로봇이 다시 시작점으로 돌아가거나, 새로운 경로가 들어올 때까지 대기
        if (dist_to_start < 1.0 && std::abs(speed) < 0.1)
        {
             // 다시 시작점으로 돌아와서 멈추면 재설정 (옵션)
             // state_ = IDLE;
             // RCLCPP_INFO(this->get_logger(), "Ready for next run...");
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