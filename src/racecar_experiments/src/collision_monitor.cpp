#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class CollisionMonitor : public rclcpp::Node
{
public:
  CollisionMonitor() : Node("collision_monitor")
  {
    // [파라미터 분리]
    // stop_threshold: 이보다 가까우면 즉시 멈춤 (0.2m)
    // go_threshold: 이보다 멀어져야 다시 출발 (0.25m) -> 노이즈 방지용 여유 공간
    this->declare_parameter("stop_threshold", 0.20);
    this->declare_parameter("go_threshold", 0.25); 

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&CollisionMonitor::scan_callback, this, std::placeholders::_1));

    pub_crash_ = this->create_publisher<std_msgs::msg::Bool>("/experiments/crash_detected", 10);

    // 초기 상태는 '안전'으로 가정
    is_crashed_state_ = false;

    RCLCPP_INFO(this->get_logger(), "Collision Monitor (Hysteresis Mode) Started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double stop_thresh = this->get_parameter("stop_threshold").as_double();
    double go_thresh = this->get_parameter("go_threshold").as_double();
    
    float min_dist = 999.0;

    // 1. 최소 거리 찾기
    for (float range : msg->ranges)
    {
      if (std::isfinite(range) && range > 0.01) 
      {
        if (range < min_dist)
        {
          min_dist = range;
        }
      }
    }

    // 2. 히스테리시스 로직 (이중 임계값)
    if (is_crashed_state_) {
        // [현재 충돌 상태임]
        // 해제되려면 'go_thresh'보다 확실히 멀어져야 함 (예: 0.25m 이상)
        if (min_dist > go_thresh) {
            is_crashed_state_ = false; // 안전 모드로 전환
            RCLCPP_INFO(this->get_logger(), "SAFE! Dist: %.3fm (> %.2fm)", min_dist, go_thresh);
        } else {
            // 여전히 0.20 ~ 0.25 사이라면 계속 충돌 상태 유지 (보수적 판단)
        }
    } 
    else {
        // [현재 안전 상태임]
        // 충돌하려면 'stop_thresh'보다 가까워져야 함 (예: 0.20m 미만)
        if (min_dist < stop_thresh) {
            is_crashed_state_ = true; // 충돌 모드로 전환
            RCLCPP_WARN(this->get_logger(), "CRASH! Dist: %.3fm (< %.2fm)", min_dist, stop_thresh);
        }
    }

    // 3. 결과 발행
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = is_crashed_state_;
    pub_crash_->publish(bool_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_crash_;
  
  bool is_crashed_state_; // 현재 상태 기억 변수
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionMonitor>());
  rclcpp::shutdown();
  return 0;
}