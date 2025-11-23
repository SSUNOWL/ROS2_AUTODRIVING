#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // 헤더 필수!

using namespace std::chrono_literals;

class StaticPathPublisher : public rclcpp::Node
{
public:
  StaticPathPublisher() : Node("static_path_publisher")
  {
    // 파라미터 설정
    this->declare_parameter("csv_path", "my_raceline.csv");
    this->declare_parameter("topic_name", "/plan");
    this->declare_parameter("frame_id", "map");

    std::string topic_name = this->get_parameter("topic_name").as_string();
    csv_path_ = this->get_parameter("csv_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // 퍼블리셔 생성
    publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
    
    // [중요] 초기화 퍼블리셔 생성
    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // CSV 로딩 및 실행
    if (load_path_from_csv()) {
      timer_ = this->create_wall_timer(
        1000ms, std::bind(&StaticPathPublisher::timer_callback, this));
      
      RCLCPP_INFO(this->get_logger(), "✅ Ready! Publishing path to '%s' every 1s.", topic_name.c_str());
      
      // 차량 위치 리셋 함수 호출
      reset_vehicle_position();
    }
  }

private:
  // ==========================================
  // [중요] 멤버 변수 선언부 (여기가 빠져서 에러가 난 겁니다)
  // ==========================================
  nav_msgs::msg::Path path_msg_;
  std::string csv_path_;
  std::string frame_id_;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  
  // ★★★ 이 줄이 없어서 에러가 났습니다! ★★★
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_; 
  
  rclcpp::TimerBase::SharedPtr timer_;

  // CSV 로딩 함수
  bool load_path_from_csv()
  {
    std::ifstream file(csv_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to open CSV file: %s", csv_path_.c_str());
      return false;
    }

    path_msg_.header.frame_id = frame_id_;
    std::string line;
    int count = 0;

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string x_str, y_str, yaw_str;

      // x, y, yaw 세 개를 읽어옴
      if (std::getline(ss, x_str, ',') && 
          std::getline(ss, y_str, ',') && 
          std::getline(ss, yaw_str, ',')) 
      {
        try {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = frame_id_;
          
          pose.pose.position.x = std::stod(x_str);
          pose.pose.position.y = std::stod(y_str);
          pose.pose.position.z = 0.0;

          // 읽어온 Yaw를 다시 쿼터니언으로 변환
          double yaw = std::stod(yaw_str);
          pose.pose.orientation.z = std::sin(yaw / 2.0);
          pose.pose.orientation.w = std::cos(yaw / 2.0);
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;

          path_msg_.poses.push_back(pose);
          count++;
        } catch (...) { continue; }
      }
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "📂 Loaded %d points with Saved Orientation.", count);
    return true;
  }

  // 타이머 콜백
  void timer_callback()
  {
    path_msg_.header.stamp = this->get_clock()->now();
    publisher_->publish(path_msg_);
  }

  // 차량 위치 리셋 함수
  void reset_vehicle_position()
  {
    if (path_msg_.poses.empty()) return;

    auto init_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_msg.header.frame_id = "map";
    init_msg.header.stamp = this->get_clock()->now();

    auto first_pose = path_msg_.poses.front();
    init_msg.pose.pose.position = first_pose.pose.position;
    init_msg.pose.pose.orientation = first_pose.pose.orientation;

    // 약간 대기 (초기화 시간 확보)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 이제 init_pose_pub_가 선언되었으므로 에러 안 남
    init_pose_pub_->publish(init_msg);
    
    RCLCPP_INFO(this->get_logger(), "🚀 Vehicle Teleported to Start Point: (%.2f, %.2f)", 
        first_pose.pose.position.x, first_pose.pose.position.y);
  }
}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticPathPublisher>());
  rclcpp::shutdown();
  return 0;
}