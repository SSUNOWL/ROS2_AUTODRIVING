#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp> 

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>

using namespace std;

struct Waypoint {
    double x;
    double y;
    double v; 
};

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit() : Node("pure_pursuit_node") {
        this->declare_parameter("csv_path", "raceline_with_speed.csv");
        this->declare_parameter("lookahead_min", 1.0);
        this->declare_parameter("lookahead_gain", 0.3);
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_speed", 8.0);
        this->declare_parameter("use_frenet_path", false);
        this->declare_parameter("frenet_path_topic", "/selected_path");

        string csv_path = this->get_parameter("csv_path").as_string();
        lookahead_min_ = this->get_parameter("lookahead_min").as_double();
        lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_speed_      = this->get_parameter("max_speed").as_double();

        use_frenet_path_ = this->get_parameter("use_frenet_path").as_bool();
        frenet_path_topic_ = this->get_parameter("frenet_path_topic").as_string();

        if (!use_frenet_path_) {
            if (!load_waypoints(csv_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file!");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Pure Pursuit using CSV waypoints: %s", csv_path.c_str());
        } else {
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                frenet_path_topic_, 10,
                std::bind(&PurePursuit::path_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Pure Pursuit using Frenet path from topic: %s", frenet_path_topic_.c_str());
        }

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit_marker", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        crash_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/experiments/crash_detected", 10,
            std::bind(&PurePursuit::crash_callback, this, std::placeholders::_1));

        // [추가됨] 목표 도달 신호 구독
        goal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/experiments/goal_reached", 10,
            std::bind(&PurePursuit::goal_callback, this, std::placeholders::_1));

        last_crash_msg_time_ = this->now();
        is_goal_reached_ = false; // [추가됨] 목표 도달 상태 초기화

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Started (Auto-Resume & Goal Stop Enabled)."); // 로그 수정
    }

private:
    vector<Waypoint> waypoints_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crash_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_sub_; // [추가됨]

    bool is_emergency_ = false;
    bool is_goal_reached_ = false; // [추가됨] 목표 도달 상태 플래그
    rclcpp::Time last_crash_msg_time_; // 타임아웃용 시간 기록

    bool use_frenet_path_;
    std::string frenet_path_topic_;

    double lookahead_min_;
    double lookahead_gain_;
    double wheelbase_;
    double max_speed_;

    // [수정됨] 충돌 신호 콜백: False 수신 시 즉시 해제 + Heartbeat
    void crash_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        last_crash_msg_time_ = this->now(); // 시간 갱신

        if (msg->data) {
            if (!is_emergency_) {
                is_emergency_ = true;
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "EMERGENCY STOP ACTIVATED! (Collision Detected)");
            }
        } else {
            if (is_emergency_) {
                is_emergency_ = false;
                RCLCPP_INFO(this->get_logger(), "Collision Cleared. Resuming Pure Pursuit...");
            }
        }
    }

    // [추가됨] 목표 도달 신호 콜백
    void goal_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            if (!is_goal_reached_) {
                is_goal_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "GOAL STOP ACTIVATED! (Goal Reached)");
            }
        } else {
            if (is_goal_reached_) {
                is_goal_reached_ = false;
                RCLCPP_INFO(this->get_logger(), "Goal Stop Cleared. Resuming Pure Pursuit...");
            }
        }
    }

    bool load_waypoints(string path) {
        ifstream file(path);
        if (!file.is_open()) return false;

          string line;
            getline(file, line); 

        while (getline(file, line)) {
            stringstream ss(line);
            string cell;
            vector<string> row;
            while (getline(ss, cell, ',')) row.push_back(cell);

            if (row.size() >= 3) {
                Waypoint wp;
                try {
                    wp.x = stod(row[0]);
                    wp.y = stod(row[1]);
                    wp.v = stod(row[2]); 
                    waypoints_.push_back(wp);
                } catch (...) { continue; }
            }
        }
        return !waypoints_.empty();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // [NEW] 타임아웃 체크 (충돌 관련만)
        rclcpp::Time now = this->now();
        double time_diff = (now - last_crash_msg_time_).seconds();

        if (is_emergency_ && time_diff > 5.0) {
            is_emergency_ = false;
            RCLCPP_INFO(this->get_logger(), "Collision Monitor Timeout. Resuming Pure Pursuit...");
        }

        // 비상 상황이거나 목표에 도달했으면 정지 [수정됨]
        if (is_emergency_ || is_goal_reached_) {
            ackermann_msgs::msg::AckermannDriveStamped stop_msg;
            stop_msg.header = msg->header;
            stop_msg.drive.steering_angle = 0.0;
            stop_msg.drive.speed = 0.0;
            drive_pub_->publish(stop_msg);
            return;
        }

        if (waypoints_.empty()) return;

        // --- 정상 주행 로직 (기존과 동일) ---
        double curr_x = msg->pose.pose.position.x;
        double curr_y = msg->pose.pose.position.y;
        double curr_vel = msg->twist.twist.linear.x;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        int best_idx = 0;
        double min_dist_sq = 1e9;
        
        for (size_t i = 0; i < waypoints_.size(); i++) {
            double dx = waypoints_[i].x - curr_x;
            double dy = waypoints_[i].y - curr_y;
            double dist_sq = dx*dx + dy*dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }

        double lookahead_dist = lookahead_min_ + lookahead_gain_ * curr_vel;
        
        int target_idx = best_idx;
        double dist_sum = 0.0;
        
        for (size_t i = 0; i < waypoints_.size(); i++) {
            int curr_i = (best_idx + i) % waypoints_.size();
            int next_i = (curr_i + 1) % waypoints_.size();
            
            double dx = waypoints_[next_i].x - waypoints_[curr_i].x;
            double dy = waypoints_[next_i].y - waypoints_[curr_i].y;
            dist_sum += std::hypot(dx, dy);

            if (dist_sum >= lookahead_dist) {
                target_idx = next_i;
                break;
            }
        }
        
        Waypoint target = waypoints_[target_idx];

        double dx = target.x - curr_x;
        double dy = target.y - curr_y;
        
        double local_x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
        double local_y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;

        double dist_to_target_sq = local_x * local_x + local_y * local_y;
        const double eps = 1e-6;

        double curvature = 0.0;
        if (dist_to_target_sq > eps) {
            curvature = 2.0 * local_y / dist_to_target_sq;
        }

        double steering_angle = std::atan(curvature * wheelbase_);

        if (!std::isfinite(steering_angle)) {
            steering_angle = 0.0;
        }

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header = msg->header;
        drive_msg.drive.steering_angle = steering_angle;

        double v_cmd = target.v;

        if (!std::isfinite(v_cmd) || v_cmd < 0.0) {
            v_cmd = 0.0;
        }

        if (max_speed_ > 0.0) {
            v_cmd = std::min(v_cmd, max_speed_);
        }

        drive_msg.drive.speed = v_cmd;
        drive_pub_->publish(drive_msg);

        publish_marker(target.x, target.y);
    }

    void publish_marker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        vis_pub_->publish(marker);
    }
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::vector<Waypoint> new_wps;
        new_wps.reserve(msg->poses.size());

        for (const auto & ps : msg->poses) {
            Waypoint wp;
            wp.x = ps.pose.position.x;
            wp.y = ps.pose.position.y;
            wp.v = ps.pose.position.z;
            new_wps.push_back(wp);
        }

        if (!new_wps.empty()) {
            waypoints_ = std::move(new_wps);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}