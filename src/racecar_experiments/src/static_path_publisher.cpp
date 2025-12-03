#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <cmath>
#include <algorithm> // for max

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

        // 자차 CSV 로딩 (여기서 0열, 1열을 명시적으로 지정)
        if (load_path_from_csv(0, 1)) {
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

    // [수정됨] 자차 경로 로딩 함수도 인덱스를 받도록 개선 (기본값 0, 1)
    bool load_path_from_csv(int x_idx = 0, int y_idx = 1)
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
            std::string cell;
            std::vector<std::string> row_data;

            // 한 줄을 쉼표로 분리하여 벡터에 저장 (유연한 인덱스 접근을 위해)
            while(std::getline(ss, cell, ',')) {
                row_data.push_back(cell);
            }

            // 지정한 인덱스가 유효한지 확인
            int max_idx = std::max(x_idx, y_idx);
            if (row_data.size() > static_cast<size_t>(max_idx)) {
                try {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = frame_id_;
                    
                    // 지정된 인덱스(x_idx, y_idx)의 값을 사용
                    pose.pose.position.x = std::stod(row_data[x_idx]);
                    pose.pose.position.y = std::stod(row_data[y_idx]);
                    
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

    // 헬퍼 함수: CSV 파일의 특정 컬럼을 읽어 시작 위치(x,y)와 방향(yaw)을 계산
    // x_idx: X좌표 컬럼 인덱스(기본 0), y_idx: Y좌표 컬럼 인덱스(기본 1)
    bool get_start_pose_from_csv(const std::string& filepath, double& out_x, double& out_y, double& out_yaw, int x_idx = 0, int y_idx = 1)
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
            std::string cell;
            std::vector<std::string> row_data;

            while(std::getline(ss, cell, ',')) {
                row_data.push_back(cell);
            }

            int max_idx = std::max(x_idx, y_idx);
            if (row_data.size() > static_cast<size_t>(max_idx)) {
                try {
                    double x = std::stod(row_data[x_idx]);
                    double y = std::stod(row_data[y_idx]);
                    points.push_back({x, y});
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
            out_yaw = 0.0;
        }
        return true;
    }

    void reset_vehicle_position()
    {
        // 1. 자차(Ego) 위치 리셋
        // load_path_from_csv(0, 1)로 이미 올바르게 로드된 path_msg_를 사용하므로 0열, 1열 데이터가 적용됨
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

        // 2. 상대차(Opponent) 위치 리셋
        if (spawn_opponent_enabled_) {
            double opp_x, opp_y, opp_yaw;
            
            // [중요] 상대차 CSV는 1열(x), 2열(y)이므로 인자를 1, 2로 지정
            if (get_start_pose_from_csv(opponent_csv_path_, opp_x, opp_y, opp_yaw, 1, 2)) {
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
                    "상대차 CSV(%s) 로드 완료 (1열, 2열 사용) -> 스폰 위치: (%.2f, %.2f, Yaw: %.2f)",
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