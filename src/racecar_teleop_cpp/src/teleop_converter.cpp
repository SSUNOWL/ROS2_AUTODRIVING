#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class TeleopConverter : public rclcpp::Node
{
public:
  TeleopConverter() : Node("teleop_converter")
  {
    // 1. 구독(Subscribe): 키보드 제어 노드에서 나오는 '/cmd_vel' 토픽을 듣습니다.
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TeleopConverter::topic_callback, this, _1));

    // 2. 발행(Publish): 시뮬레이터가 듣고 있는 '/drive' 토픽으로 변환해서 보냅니다.
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    
    RCLCPP_INFO(this->get_logger(), "Teleop Converter Node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to /cmd_vel -> Publishing to /drive");
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    // 변환 로직
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

    // Twist의 linear.x (전진 속도) -> Ackermann의 speed
    drive_msg.drive.speed = msg->linear.x;

    // Twist의 angular.z (회전 속도) -> Ackermann의 steering_angle
    // (키보드 입력값을 적절한 조향각으로 매핑합니다)
    drive_msg.drive.steering_angle = msg->angular.z;

    // 메시지 발행
    publisher_->publish(drive_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopConverter>());
  rclcpp::shutdown();
  return 0;
}