// 파일명: planner_mux_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class LocalPlannerMux : public rclcpp::Node
{
public:
  LocalPlannerMux() : Node("local_planner_mux")
  {
    // 구독 (토픽 이름은 실제 환경에 맞게 추후 수정 가능)
    sub_frenet_ = this->create_subscription<nav_msgs::msg::Path>(
      "/frenet_local_plan", 10, std::bind(&LocalPlannerMux::frenet_callback, this, _1));

    sub_fgm_ = this->create_subscription<nav_msgs::msg::Path>(
      "/fgm_path", 10, std::bind(&LocalPlannerMux::fgm_callback, this, _1));

    // 발행 (Pure Pursuit가 구독할 토픽)
    pub_selected_ = this->create_publisher<nav_msgs::msg::Path>(
      "/selected_path", 10);

    // 20Hz 주기로 제어 루프 실행
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&LocalPlannerMux::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Mux Node Started: Listening to /frenet_local_plan and /fgm_path");
  }

private:
  nav_msgs::msg::Path latest_frenet_path_;
  nav_msgs::msg::Path latest_fgm_path_;
  bool has_frenet_ = false;
  bool has_fgm_ = false;

  void frenet_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    latest_frenet_path_ = *msg;
    has_frenet_ = true;
  }

  void fgm_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    latest_fgm_path_ = *msg;
    has_fgm_ = true;
  }

  void control_loop() {
    nav_msgs::msg::Path selected_path;
    bool publish_needed = false;

    // 두 경로 모두 수신 시 비교 로직 수행
    if (has_frenet_ && has_fgm_) {
      if (compare_paths(latest_frenet_path_, latest_fgm_path_) == 0) {
        selected_path = latest_frenet_path_;
        // RCLCPP_INFO(this->get_logger(), "Selected: Frenet");
      } else {
        selected_path = latest_fgm_path_;
        // RCLCPP_INFO(this->get_logger(), "Selected: FGM");
      }
      publish_needed = true;
    }
    else if (has_frenet_) {
      selected_path = latest_frenet_path_;
      publish_needed = true;
    }
    else if (has_fgm_) {
      selected_path = latest_fgm_path_;
      publish_needed = true;
    }

    if (publish_needed) {
      pub_selected_->publish(selected_path);
    }
  }

  // 0: Frenet, 1: FGM
  int compare_paths(const nav_msgs::msg::Path& frenet, const nav_msgs::msg::Path& fgm) {
    // [간단 로직] Frenet 경로가 너무 짧으면(생성 실패 추정) FGM 사용
    if (frenet.poses.size() < 3) return 1; 
    return 0; // 기본은 Frenet
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_frenet_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_fgm_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_selected_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerMux>());
  rclcpp::shutdown();
  return 0;
}