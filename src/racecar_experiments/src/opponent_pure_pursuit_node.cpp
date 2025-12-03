#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>
#include <algorithm>

using namespace std;

struct Waypoint {
    double t;   // [NEW] 시간 정보 추가
    double x;
    double y;
    double yaw; 
    double v; 
};

class OpponentPurePursuit : public rclcpp::Node {
public:
    OpponentPurePursuit() : Node("opponent_pure_pursuit_node") {
        // --- 파라미터 ---
        this->declare_parameter("csv_path", "spielberg_opponent_scenarioA.csv");
        this->declare_parameter("lookahead_min", 0.5); 
        this->declare_parameter("lookahead_gain", 0.4); 
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_speed", 100.0); // CSV 본연의 속도를 내기 위해 제한 품
        
        this->declare_parameter("odom_topic", "/opp_racecar/odom");
        this->declare_parameter("drive_topic", "/opp_drive");
        this->declare_parameter("vis_topic", "/opp_marker");

        string csv_path = this->get_parameter("csv_path").as_string();
        lookahead_min_  = this->get_parameter("lookahead_min").as_double();
        lookahead_gain_ = this->get_parameter("lookahead_gain").as_double();
        wheelbase_      = this->get_parameter("wheelbase").as_double();
        max_speed_      = this->get_parameter("max_speed").as_double();
        
        string odom_topic = this->get_parameter("odom_topic").as_string();
        string drive_topic = this->get_parameter("drive_topic").as_string();
        string vis_topic = this->get_parameter("vis_topic").as_string();

        // --- CSV 로드 ---
        if (!load_waypoints(csv_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load CSV: %s", csv_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints with TIME info from: %s", waypoints_.size(), csv_path.c_str());
        }

        // --- 초기화 ---
        last_closest_idx_ = 0;
        start_time_initialized_ = false; // [NEW] 시간 초기화 플래그

        // --- Pub/Sub ---
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(vis_topic, 10);

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 
            qos,
            std::bind(&OpponentPurePursuit::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Opponent Node Ready. Time-Synchronized Mode.");
    }

private:
    vector<Waypoint> waypoints_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    double lookahead_min_;
    double lookahead_gain_;
    double wheelbase_;
    double max_speed_;

    int last_closest_idx_;
    
    // [NEW] 시간 동기화 관련 변수
    rclcpp::Time start_time_;
    bool start_time_initialized_;

    bool load_waypoints(string path) {
        ifstream file(path);
        if (!file.is_open()) return false;

        string line;
        waypoints_.clear();
        
        while (getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            stringstream ss(line);
            string cell;
            vector<string> row;
            while (getline(ss, cell, ',')) row.push_back(cell);

            Waypoint wp;
            try {
                // time, x, y, yaw, speed
                if (row.size() >= 5) {
                    wp.t = stod(row[0]); // [NEW] 시간 파싱
                    wp.x = stod(row[1]);
                    wp.y = stod(row[2]);
                    wp.yaw = stod(row[3]);
                    wp.v = stod(row[4]);
                    waypoints_.push_back(wp);
                }
            } catch (...) { continue; }
        }
        return !waypoints_.empty();
    }

    // 현재 경과 시간(elapsed)에 해당하는 목표 속도를 찾는 함수
    double get_speed_at_time(double elapsed_time) {
        if (waypoints_.empty()) return 0.0;
        
        // CSV의 마지막 시간보다 더 지났으면 멈춤 (혹은 루프)
        if (elapsed_time > waypoints_.back().t) return 0.0;
        if (elapsed_time < waypoints_.front().t) return 0.0; // 아직 시작 전

        // 이진 탐색 등으로 최적화할 수 있지만, 데이터가 순차적이므로 간단히 구현
        // (더 정확하려면 lower_bound 사용 추천)
        auto it = std::lower_bound(waypoints_.begin(), waypoints_.end(), elapsed_time, 
            [](const Waypoint& wp, double t) {
                return wp.t < t;
            });

        if (it == waypoints_.end()) return waypoints_.back().v;
        if (it == waypoints_.begin()) return waypoints_.front().v;

        // 보간(Interpolation)은 굳이 안 해도 됨 (데이터가 촘촘하므로)
        return it->v;
    }

    double get_dist_sq(const Waypoint& wp, double x, double y) {
        return (wp.x - x)*(wp.x - x) + (wp.y - y)*(wp.y - y);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (waypoints_.empty()) return;

        // [NEW] 첫 실행 시 시작 시간 기록
        if (!start_time_initialized_) {
            start_time_ = this->now();
            start_time_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Race Start! Timer Synchronized.");
        }

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

        // 1. 위치 기반으로 '가장 가까운 점' 찾기 (조향용)
        int search_range = 500;
        int closest_idx = last_closest_idx_;
        double min_dist_sq = 1e9;
        int n_points = waypoints_.size();

        for (int i = 0; i < search_range; i++) {
            int idx = (last_closest_idx_ + i) % n_points;
            double d = get_dist_sq(waypoints_[idx], curr_x, curr_y);
            if (d < min_dist_sq) {
                min_dist_sq = d;
                closest_idx = idx;
            }
        }
        last_closest_idx_ = closest_idx;

        // 2. Lookahead Point 찾기
        double lookahead_dist = lookahead_min_ + lookahead_gain_ * curr_vel;
        int target_idx = closest_idx;
        double dist_acc = 0.0;
        
        for (int i = 0; i < n_points; i++) {
            int curr_i = (closest_idx + i) % n_points;
            int next_i = (curr_i + 1) % n_points;
            double dx = waypoints_[next_i].x - waypoints_[curr_i].x;
            double dy = waypoints_[next_i].y - waypoints_[curr_i].y;
            dist_acc += std::hypot(dx, dy);

            if (dist_acc >= lookahead_dist) {
                target_idx = next_i;
                break;
            }
        }
        Waypoint target_point = waypoints_[target_idx];

        // 3. Pure Pursuit 조향각 계산
        double dx = target_point.x - curr_x;
        double dy = target_point.y - curr_y;
        double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        double L_sq = local_x * local_x + local_y * local_y;
        
        double steering_angle = 0.0;
        if (L_sq > 1e-6) {
            steering_angle = std::atan2(2.0 * wheelbase_ * local_y, L_sq);
        }

        // 4. [핵심] 속도는 '시간'을 기준으로 결정 (Replay 방식)
        double elapsed_sec = (this->now() - start_time_).seconds();
        double time_based_speed = get_speed_at_time(elapsed_sec);

        // 최대 속도 제한
        if (max_speed_ > 0.0) {
            time_based_speed = std::min(time_based_speed, max_speed_);
        }
        if (time_based_speed < 0.0) time_based_speed = 0.0;

        // 5. 명령 발행
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header = msg->header;
        drive_msg.header.frame_id = "map";
        drive_msg.drive.steering_angle = steering_angle;
        
        // 여기서 위치 기반 target.v가 아니라 시간 기반 speed를 넣음!
        drive_msg.drive.speed = time_based_speed; 
        
        drive_pub_->publish(drive_msg);

        publish_marker(target_point.x, target_point.y);
    }

    void publish_marker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "opponent_pp";
        marker.id = 100;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.3;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0; 
        marker.color.b = 0.0;
        vis_pub_->publish(marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpponentPurePursuit>());
    rclcpp::shutdown();
    return 0;
}