#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class CollisionMonitor : public rclcpp::Node
{
public:
  CollisionMonitor() : Node("collision_monitor")
  {
    // 파라미터 선언 (기본값: 0.2m)
    // 실험 때마다 재빌드 없이 'ros2 run ... --ros-args -p threshold:=0.3' 처럼 변경 가능
    this->declare_parameter("threshold", 0.2);

    // 라이다 구독
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&CollisionMonitor::scan_callback, this, std::placeholders::_1));

    // 충돌 상태 발행 (True: 충돌함, False: 안전함)
    pub_crash_ = this->create_publisher<std_msgs::msg::Bool>("/experiments/crash_detected", 10);

    RCLCPP_INFO(this->get_logger(), "Experiment Collision Monitor Started.");
    RCLCPP_INFO(this->get_logger(), "Trigger Distance: %.2fm", this->get_parameter("threshold").as_double());
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double threshold = this->get_parameter("threshold").as_double();
    bool is_crashed = false;
    float min_dist = 999.0;

    // 라이다 데이터 전체 순회
    for (float range : msg->ranges)
    {
      // 유효한 거리 값인지 확인 (inf, NaN, 0.0 제외)
      if (std::isfinite(range) && range > 0.01) 
      {
        if (range < min_dist)
        {
          min_dist = range;
        }
      }
    }

    // 충돌 판단
    if (min_dist < threshold)
    {
      is_crashed = true;
      
      // 로그가 너무 많이 뜨는 것을 방지하기 위해 1초에 한 번만 출력 (Throttle)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "CRASH! Min Dist: %.3fm (Threshold: %.2fm)", min_dist, threshold);
    }

    // 결과 발행
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = is_crashed;
    pub_crash_->publish(bool_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_crash_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionMonitor>());
  rclcpp::shutdown();
  return 0;
}