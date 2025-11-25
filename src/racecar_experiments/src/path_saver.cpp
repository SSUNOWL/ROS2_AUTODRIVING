#include <fstream>
#include <iostream>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

class PathSaver : public rclcpp::Node
{
public:
  PathSaver() : Node("path_saver")
  {
    // 저장할 파일 이름 파라미터 (기본값: path_segment.csv)
    this->declare_parameter("filename", "path_segment.csv");

    // 전역 경로 구독
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, std::bind(&PathSaver::path_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Path Saver Waiting for /plan topic...");
  }

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path!");
      return;
    }

    std::string filename = this->get_parameter("filename").as_string();
    std::ofstream file(filename);
    
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return;
    }

    // 점들의 개수
    size_t path_size = msg->poses.size();

    for (size_t i = 0; i < path_size; ++i) {
      double current_x = msg->poses[i].pose.position.x;
      double current_y = msg->poses[i].pose.position.y;
      double yaw = 0.0;

      if (i < path_size - 1) {
        // 다음 점이 있는 경우: 다음 점을 바라보는 각도 계산
        double next_x = msg->poses[i+1].pose.position.x;
        double next_y = msg->poses[i+1].pose.position.y;
        double dx = next_x - current_x;
        double dy = next_y - current_y;
        
        // 아크탄젠트로 각도(라디안) 계산
        yaw = std::atan2(dy, dx);
      } 
      else {
        // 마지막 점인 경우: 바로 이전 점의 각도를 그대로 사용 (이전 점이 없으면 0)
        // (마지막에 멈춰 있을 때 방향이 홱 돌아가는 것 방지)
        // 하지만 여기서는 파일에 저장된 직전 값을 가져올 수 없으므로 
        // 루프 안에서 간단히 처리하기 위해 이전 계산 방식을 유지할 수 없으니 0으로 두거나, 
        // 그냥 i-1 번째 점과 계산하면 됩니다.
        if (i > 0) {
             double prev_x = msg->poses[i-1].pose.position.x;
             double prev_y = msg->poses[i-1].pose.position.y;
             yaw = std::atan2(current_y - prev_y, current_x - prev_x);
        }
      }

      // x, y, 계산된_yaw 저장
      file << current_x << "," << current_y << "," << yaw << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Path SAVED to %s (%zu points) with CALCULATED YAW.", filename.c_str(), path_size);
    
    // 저장 완료 후 종료
    rclcpp::shutdown();
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSaver>());
  rclcpp::shutdown();
  return 0;
}