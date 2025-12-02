#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

class PathTimer : public rclcpp::Node
{
public:
    PathTimer() : Node("path_timer"), state_(IDLE), has_path_(false), has_left_start_(false)
    {
        // 파라미터: 도착 판정 거리 (기본 0.5m)
        this->declare_parameter("goal_tolerance", 0.5);

        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathTimer::path_callback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&PathTimer::odom_callback, this, std::placeholders::_1));

        pub_time_ = this->create_publisher<std_msgs::msg::Float32>("/experiments/travel_time", 10);
        pub_goal_ = this->create_publisher<std_msgs::msg::Bool>("/experiments/goal_reached", 10);

        // [추가됨] 종료 타이머 설정
        shutdown_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms 마다 체크
            std::bind(&PathTimer::check_for_shutdown, this));
        shutdown_timer_->cancel(); // 초기에는 비활성화

        RCLCPP_INFO(this->get_logger(), "Path Timer Ready. Waiting for Global Plan...");
    }

private:
    enum State { IDLE, RUNNING, FINISHED, SHUTTING_DOWN };
    State state_;
    bool has_path_;
    bool has_left_start_; 
    double start_x_, start_y_;
    double goal_x_, goal_y_;
    rclcpp::Time start_time_;
    rclcpp::Time goal_time_; // 목표 도달 시간 기록

    rclcpp::TimerBase::SharedPtr shutdown_timer_; // 종료 타이머 핸들
    
    // [추가됨] 노드 종료를 위한 주기적 체크 함수
    void check_for_shutdown()
    {
        // 목표 도달 후 5초가 지나면 노드를 종료합니다.
        if (state_ == SHUTTING_DOWN) {
            double elapsed_since_goal = (this->get_clock()->now() - goal_time_).seconds();
            if (elapsed_since_goal >= 5.0) { // 5초 쿨다운
                RCLCPP_WARN(this->get_logger(), "Goal reached 5s ago. Shutting down Path Timer node.");
                rclcpp::shutdown(); // ROS 2 노드 종료
            }
        }
    }
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // 노드가 이미 종료 중이면 새 경로를 무시합니다.
        if (state_ == SHUTTING_DOWN) return; 
        
        if (msg->poses.empty() || msg->poses.size() < 10) {
            if (msg->poses.empty()) RCLCPP_WARN(this->get_logger(), "Received empty path!");
            else RCLCPP_WARN(this->get_logger(), "Path is too short (points < 10). Ignoring.");
            return; 
        }

        double sx = msg->poses.front().pose.position.x;
        double sy = msg->poses.front().pose.position.y;
        double gx = msg->poses.back().pose.position.x;
        double gy = msg->poses.back().pose.position.y;
        
        if (state_ == IDLE || state_ == FINISHED)
        {
            start_x_ = sx;
            start_y_ = sy;
            state_ = IDLE;
            has_left_start_ = false;

            auto goal_msg = std_msgs::msg::Bool();
            goal_msg.data = false;
            pub_goal_->publish(goal_msg);
            
            RCLCPP_INFO(this->get_logger(), "New Path Set! Start: (%.2f, %.2f) -> Goal: (%.2f, %.2f)", 
                start_x_, start_y_, gx, gy);
        }

        goal_x_ = gx;
        goal_y_ = gy;
        has_path_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!has_path_ || state_ == SHUTTING_DOWN) return; // 종료 중이면 무시

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double speed = msg->twist.twist.linear.x;
        double tolerance = this->get_parameter("goal_tolerance").as_double();

        double dist_to_start = std::sqrt(std::pow(x - start_x_, 2) + std::pow(y - start_y_, 2));
        double dist_to_goal = std::sqrt(std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));

        switch (state_)
        {
            case IDLE:
                if (dist_to_start < 2.0 && std::abs(speed) > 0.1)
                {
                    start_time_ = this->get_clock()->now();
                    state_ = RUNNING;
                    has_left_start_ = false; 
                    RCLCPP_INFO(this->get_logger(), "Timer Started! GO GO GO!");
                }
                break;

            case RUNNING:
            {
                if (!has_left_start_) {
                    if (dist_to_start > 3.0) {
                        has_left_start_ = true;
                        RCLCPP_INFO(this->get_logger(), "Vehicle left start zone. Goal check enabled.");
                    }
                }

                if (has_left_start_ && dist_to_goal < tolerance)
                {
                    double elapsed = (this->get_clock()->now() - start_time_).seconds();
                    state_ = SHUTTING_DOWN; // [수정됨] FINISHED 대신 SHUTTING_DOWN 상태로 전환
                    goal_time_ = this->get_clock()->now(); // 목표 도달 시간 기록

                    RCLCPP_INFO(this->get_logger(), "GOAL REACHED! Time: %.4f seconds", elapsed);
                    
                    auto time_msg = std_msgs::msg::Float32();
                    time_msg.data = elapsed;
                    pub_time_->publish(time_msg);

                    auto goal_msg = std_msgs::msg::Bool();
                    goal_msg.data = true;
                    pub_goal_->publish(goal_msg);
                    
                    // [추가됨] 종료 타이머 시작
                    shutdown_timer_->reset();
                }
                break;
            }

            case FINISHED:
            case SHUTTING_DOWN:
                break; // 오도메트리 콜백에서는 추가 작업을 하지 않습니다.
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_time_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathTimer>());
    rclcpp::shutdown();
    return 0;
}