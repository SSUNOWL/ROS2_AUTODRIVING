#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <cmath>

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
        // 1. 공통 파라미터
        this->declare_parameter("csv_path", "my_raceline.csv");
        this->declare_parameter("topic_name", "/plan");
        this->declare_parameter("frame_id", "map");

        // 2. 상대 차량(Opponent) 관련 파라미터
        // spawn_opponent_enabled가 true이면 opponent_csv_path를 읽어서 위치를 잡습니다.
        this->declare_parameter("spawn_opponent_enabled", false);
        this->declare_parameter("opponent_csv_path", "opponent_raceline.csv"); 
        this->declare_parameter("opponent_spawn_topic", "/goal_pose");

        std::string topic_name = this->get_parameter("topic_name").as_string();
        csv_path_ = this->get_parameter("csv_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        spawn_opponent_enabled_ = this->get_parameter("spawn_opponent_enabled").as_bool();
        opponent_csv_path_ = this->get_parameter("opponent_csv_path").as_string();
        std::string opponent_topic_name = this->get_parameter("opponent_spawn_topic").as_string();

        // 퍼블리셔 생성
        publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name, 10);
        init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        opponent_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(opponent_topic_name, 10);

        // 자차 CSV 로딩
        if (load_path_from_csv()) {
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&StaticPathPublisher::timer_callback, this));
            
            RCLCPP_INFO(this->get_logger(), "준비 완료! '%s' 토픽으로 경로를 발행합니다.", topic_name.c_str());
            
            // 차량 위치 리셋 (자차 및 상대차)
            reset_vehicle_position();
        }
    }

private:
    nav_msgs::msg::Path path_msg_;
    std::string csv_path_;
    std::string frame_id_;
    
    // 상대 차량 파라미터
    bool spawn_opponent_enabled_;
    std::string opponent_csv_path_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_; 
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr opponent_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // CSV 로딩 (자차 경로용 - 전체 로드)
    bool load_path_from_csv()
    {
        std::ifstream file(csv_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "자차 CSV 파일을 열 수 없습니다: %s", csv_path_.c_str());
            return false;
        }

        path_msg_.header.frame_id = frame_id_;
        std::string line;
        int count = 0;

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                try {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = frame_id_;
                    pose.pose.position.x = std::stod(x_str);
                    pose.pose.position.y = std::stod(y_str);
                    // Yaw는 경로 추종 시 Pure Pursuit이 계산하므로 여기선 0 또는 CSV 값 사용
                    // (생략: CSV 파싱 로직 유지)
                    path_msg_.poses.push_back(pose);
                    count++;
                } catch (...) { continue; }
            }
        }
        file.close();
        return true;
    }

    void timer_callback()
    {
        path_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(path_msg_);
    }

    // 헬퍼 함수: CSV 파일의 앞 2개 점을 읽어 시작 위치(x,y)와 방향(yaw)을 계산
    bool get_start_pose_from_csv(const std::string& filepath, double& out_x, double& out_y, double& out_yaw)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        std::string line;
        struct Point { double x, y; };
        std::vector<Point> points;

        // 첫 2줄만 읽음
        int lines_read = 0;
        while (std::getline(file, line) && lines_read < 2) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                try {
                    points.push_back({std::stod(x_str), std::stod(y_str)});
                    lines_read++;
                } catch(...) {}
            }
        }
        file.close();

        if (points.size() < 1) return false;

        // 위치 설정
        out_x = points[0].x;
        out_y = points[0].y;

        // Yaw 계산
        if (points.size() >= 2) {
            double dx = points[1].x - points[0].x;
            double dy = points[1].y - points[0].y;
            out_yaw = std::atan2(dy, dx);
        } else {
            out_yaw = 0.0; // 점이 하나뿐이면 0도
        }
        return true;
    }

    void reset_vehicle_position()
    {
        // 1. 자차(Ego) 위치 리셋 (경로 데이터 기반)
        if (!path_msg_.poses.empty()) {
            auto init_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            init_msg.header.frame_id = frame_id_;
            init_msg.header.stamp = this->get_clock()->now();

            auto first_pose = path_msg_.poses[0];
            init_msg.pose.pose.position = first_pose.pose.position;

            double yaw_calc = 0.0;
            if (path_msg_.poses.size() >= 2) {
                double dx = path_msg_.poses[1].pose.position.x - path_msg_.poses[0].pose.position.x;
                double dy = path_msg_.poses[1].pose.position.y - path_msg_.poses[0].pose.position.y;
                yaw_calc = std::atan2(dy, dx);
            }
            init_msg.pose.pose.orientation.z = std::sin(yaw_calc / 2.0);
            init_msg.pose.pose.orientation.w = std::cos(yaw_calc / 2.0);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            init_pose_pub_->publish(init_msg);
            RCLCPP_INFO(this->get_logger(), "자차 초기화: (%.2f, %.2f, %.2f rad)", 
                first_pose.pose.position.x, first_pose.pose.position.y, yaw_calc);
        }

        // 2. 상대차(Opponent) 위치 리셋 (CSV 기반 자동 계산)
        if (spawn_opponent_enabled_) {
            double opp_x, opp_y, opp_yaw;
            if (get_start_pose_from_csv(opponent_csv_path_, opp_x, opp_y, opp_yaw)) {
                auto opponent_msg = geometry_msgs::msg::PoseStamped();
                opponent_msg.header.frame_id = frame_id_;
                opponent_msg.header.stamp = this->get_clock()->now();

                opponent_msg.pose.position.x = opp_x;
                opponent_msg.pose.position.y = opp_y;
                opponent_msg.pose.position.z = 0.0;

                opponent_msg.pose.orientation.z = std::sin(opp_yaw / 2.0);
                opponent_msg.pose.orientation.w = std::cos(opp_yaw / 2.0);

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                opponent_pose_pub_->publish(opponent_msg);

                RCLCPP_WARN(this->get_logger(),
                    "상대차 CSV(%s) 로드 완료 -> 스폰 위치: (%.2f, %.2f, Yaw: %.2f)",
                    opponent_csv_path_.c_str(), opp_x, opp_y, opp_yaw);
            } else {
                RCLCPP_ERROR(this->get_logger(), "상대차 CSV 파일을 읽을 수 없습니다: %s", opponent_csv_path_.c_str());
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "상대차 스폰 비활성화 (Mode 1)");
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