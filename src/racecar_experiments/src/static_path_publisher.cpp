#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <cmath> // atan2, sin, cos 사용을 위해 필수

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class StaticPathPublisher : public rclcpp::Node
{
public:
	StaticPathPublisher() : Node("static_path_publisher")
	{
		// 1. 공통 파라미터 설정
		this->declare_parameter("csv_path", "my_raceline.csv");
		this->declare_parameter("topic_name", "/plan");
		this->declare_parameter("frame_id", "map");

		// 2. 상대 차량(Opponent) 스폰 관련 파라미터 설정
		this->declare_parameter("spawn_opponent_enabled", false);
		this->declare_parameter("opponent_x", 0.0);
		this->declare_parameter("opponent_y", 0.0);
		this->declare_parameter("opponent_yaw", 0.0);
		this->declare_parameter("opponent_spawn_topic", "/goal_pose"); // 상대 차량 스폰 토픽

		std::string topic_name = this->get_parameter("topic_name").as_string();
		csv_path_ = this->get_parameter("csv_path").as_string();
		frame_id_ = this->get_parameter("frame_id").as_string();
		
		spawn_opponent_enabled_ = this->get_parameter("spawn_opponent_enabled").as_bool();
		opponent_x_ = this->get_parameter("opponent_x").as_double();
		opponent_y_ = this->get_parameter("opponent_y").as_double();
		opponent_yaw_ = this->get_parameter("opponent_yaw").as_double();
		std::string opponent_topic_name = this->get_parameter("opponent_spawn_topic").as_string();

		// 퍼블리셔 생성 (자차 경로)
		publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
		
		// 초기화 퍼블리셔 생성 (자차 초기 위치: /initialpose)
		init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
		
		// 상대 차량 초기화 퍼블리셔 생성 (상대 차량 초기 위치)
    opponent_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(opponent_topic_name, 10);

		// CSV 로딩 및 실행
		if (load_path_from_csv()) {
			timer_ = this->create_wall_timer(
				1000ms, std::bind(&StaticPathPublisher::timer_callback, this));
			
			RCLCPP_INFO(this->get_logger(), "준비 완료! '%s' 토픽으로 경로를 1초마다 발행합니다.", topic_name.c_str());
			
			// 차량 위치 리셋 함수 호출 (자차 및 상대차)
			reset_vehicle_position();
		}
	}

private:
	nav_msgs::msg::Path path_msg_;
	std::string csv_path_;
	std::string frame_id_;
	
	// 상대 차량 파라미터 저장
	bool spawn_opponent_enabled_;
	double opponent_x_;
	double opponent_y_;
	double opponent_yaw_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_; 
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr opponent_pose_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// CSV 로딩 함수 (기존과 동일)
	bool load_path_from_csv()
	{
		std::ifstream file(csv_path_);
		if (!file.is_open()) {
			RCLCPP_ERROR(this->get_logger(), "CSV 파일을 열 수 없습니다: %s", csv_path_.c_str());
			return false;
		}

		path_msg_.header.frame_id = frame_id_;
		std::string line;
		int count = 0;

		while (std::getline(file, line)) {
			std::stringstream ss(line);
			std::string x_str, y_str, yaw_str;

			// x, y, yaw 읽기
			if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) 
			{
				try {
					geometry_msgs::msg::PoseStamped pose;
					pose.header.frame_id = frame_id_;
					
					pose.pose.position.x = std::stod(x_str);
					pose.pose.position.y = std::stod(y_str);
					pose.pose.position.z = 0.0;

					double yaw = 0.0;
					if (std::getline(ss, yaw_str, ',')) {
						// CSV에 yaw가 있는 경우 사용
						try { yaw = std::stod(yaw_str); } catch (...) { yaw = 0.0; }
					}
					
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
		RCLCPP_INFO(this->get_logger(), "%d개의 포인트를 로드했습니다.", count);
		return true;
	}

	// 타이머 콜백 (기존과 동일)
	void timer_callback()
	{
		path_msg_.header.stamp = this->get_clock()->now();
		publisher_->publish(path_msg_);
	}

	// 차량 위치 리셋 함수 (자차 및 상대차 처리 추가)
	void reset_vehicle_position()
	{
		// 1. 자차(Ego Vehicle) 위치 리셋 (기존 로직)
		if (!path_msg_.poses.empty()) {
			auto init_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
			init_msg.header.frame_id = frame_id_;
			init_msg.header.stamp = this->get_clock()->now();

			auto first_pose = path_msg_.poses[0];
			init_msg.pose.pose.position = first_pose.pose.position;

			// Yaw 계산: 첫 번째 점과 두 번째 점 사이의 각도 계산
			if (path_msg_.poses.size() >= 2) {
				double x1 = path_msg_.poses[0].pose.position.x;
				double y1 = path_msg_.poses[0].pose.position.y;
				double x2 = path_msg_.poses[1].pose.position.x;
				double y2 = path_msg_.poses[1].pose.position.y;

				double dx = x2 - x1;
				double dy = y2 - y1;
				double yaw_calc = std::atan2(dy, dx); // 라디안 각도 계산

				// 쿼터니언 변환
				init_msg.pose.pose.orientation.z = std::sin(yaw_calc / 2.0);
				init_msg.pose.pose.orientation.w = std::cos(yaw_calc / 2.0);
				init_msg.pose.pose.orientation.x = 0.0;
				init_msg.pose.pose.orientation.y = 0.0;

				RCLCPP_INFO(this->get_logger(), "계산된 자차 시작 Yaw: %.2f rad", yaw_calc);
			} else {
				// 점이 하나뿐이면 CSV에 있던 기존 값 사용
				init_msg.pose.pose.orientation = first_pose.pose.orientation;
			}
			
			// 약간 대기 (초기화 시간 확보)
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			init_pose_pub_->publish(init_msg);
			RCLCPP_INFO(this->get_logger(), "자차 시작 지점으로 이동: (%.2f, %.2f)", 
				first_pose.pose.position.x, first_pose.pose.position.y);
		}


		// 2. 상대차(Opponent) 위치 리셋 (추가된 로직)
		if (spawn_opponent_enabled_) {
			auto opponent_msg = geometry_msgs::msg::PoseStamped();
      opponent_msg.header.frame_id = frame_id_;
      opponent_msg.header.stamp = this->get_clock()->now();

      // 위치
      opponent_msg.pose.position.x = opponent_x_;
      opponent_msg.pose.position.y = opponent_y_;
      opponent_msg.pose.position.z = 0.0;

      // Yaw -> quaternion
      opponent_msg.pose.orientation.x = 0.0;
      opponent_msg.pose.orientation.y = 0.0;
      opponent_msg.pose.orientation.z = std::sin(opponent_yaw_ / 2.0);
      opponent_msg.pose.orientation.w = std::cos(opponent_yaw_ / 2.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      opponent_pose_pub_->publish(opponent_msg);

      RCLCPP_WARN(this->get_logger(),
          "상대차를 설정된 위치로 스폰/이동합니다: (%.2f, %.2f, Yaw: %.2f)",
          opponent_x_, opponent_y_, opponent_yaw_);
		} else {
			// 기능이 비활성화되면 아무 일도 하지 않습니다.
			RCLCPP_INFO(this->get_logger(), "상대차 스폰 기능이 비활성화되었습니다. (gym_bridge의 sim.yaml 설정을 따름)");
		}
	}
}; 

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StaticPathPublisher>());
	rclcpp::shutdown();
	return 0;
}