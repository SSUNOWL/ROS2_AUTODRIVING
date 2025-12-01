#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

struct Gap {
    int start_idx;
    int end_idx;
    int len;
    double center_angle;
};

class FGMNode : public rclcpp::Node {
public:
    FGMNode() : Node("fgm_node") {
        // --- [1] 안전 및 인식 파라미터 ---
        this->declare_parameter("gap_threshold", 1.2);
        this->declare_parameter("bubble_radius", 0.45);
        this->declare_parameter("fov_angle", 180.0);
        
        // --- [2] 경로 생성 파라미터 (Planning) ---
        this->declare_parameter("min_planning_dist", 2.0);
        this->declare_parameter("max_planning_dist", 4.0);
        this->declare_parameter("planning_gain", 0.7);
        
        this->declare_parameter("min_lookahead", 1.5);
        this->declare_parameter("max_lookahead", 3.0);
        this->declare_parameter("lookahead_gain", 0.5);

        // --- [3] 속도 제어 파라미터 ---
        this->declare_parameter("max_speed", 6.0);       
        this->declare_parameter("min_speed", 2.5);       
        this->declare_parameter("slow_down_dist", 4.0);  

        // 파라미터 변수 매핑
        gap_threshold_ = this->get_parameter("gap_threshold").as_double();
        bubble_radius_ = this->get_parameter("bubble_radius").as_double();
        
        min_plan_dist_ = this->get_parameter("min_planning_dist").as_double();
        max_plan_dist_ = this->get_parameter("max_planning_dist").as_double();
        plan_gain_     = this->get_parameter("planning_gain").as_double();

        min_look_ = this->get_parameter("min_lookahead").as_double();
        max_look_ = this->get_parameter("max_lookahead").as_double();
        look_gain_ = this->get_parameter("lookahead_gain").as_double();
        
        double fov_deg = this->get_parameter("fov_angle").as_double();
        fov_rad_ = (fov_deg / 2.0) * (M_PI / 180.0);

        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        slow_down_dist_ = this->get_parameter("slow_down_dist").as_double();

        // TF & Sub/Pub
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FGMNode::scan_callback, this, std::placeholders::_1));
        
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&FGMNode::path_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&FGMNode::odom_callback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/fgm_path", 10);

        RCLCPP_INFO(this->get_logger(), "FGM Robust Mode Started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    nav_msgs::msg::Path global_path_;
    bool has_global_path_ = false;
    double current_speed_ = 0.0;

    double gap_threshold_, bubble_radius_, fov_rad_;
    double min_plan_dist_, max_plan_dist_, plan_gain_;
    double min_look_, max_look_, look_gain_;
    double max_speed_, min_speed_, slow_down_dist_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        global_path_ = *msg;
        has_global_path_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. 라이다 데이터 정리 (Side Filter 포함)
        vector<double> ranges = clean_ranges(msg);

        // 2. 가장 가까운 장애물 거리 찾기 (속도 조절용)
        double min_obs_dist = 100.0;
        for (double r : ranges) {
            if (r > 0.05 && r < min_obs_dist) {
                min_obs_dist = r;
            }
        }

        // 3. 목표 속도 계산 (Adaptive Speed Logic)
        double target_speed = max_speed_;
        if (min_obs_dist < slow_down_dist_) {
            double ratio = (min_obs_dist - bubble_radius_) / (slow_down_dist_ - bubble_radius_);
            ratio = std::clamp(ratio, 0.0, 1.0);
            target_speed = min_speed_ + (max_speed_ - min_speed_) * ratio;
        }

        // 4. [NEW] 동적 버블 및 Gap 찾기
        apply_bubble(ranges, msg->angle_increment);
        vector<Gap> gaps = find_gaps(ranges, msg->angle_min, msg->angle_increment);
        
        // 동적 파라미터 계산
        double dyn_plan_dist = std::clamp(min_plan_dist_ + plan_gain_ * current_speed_, min_plan_dist_, max_plan_dist_);
        double dyn_lookahead = std::clamp(min_look_ + look_gain_ * current_speed_, min_look_, max_look_);

        if (gaps.empty()) {
            // 비상시: 최저 속도로 직진 시도하거나 멈춤
            publish_local_path_as_global(0.0, dyn_plan_dist, 0.0); // 틈 없으면 정지
            return;
        }

        // 5. 최적 각도 선정 (안전 로직 추가됨)
        double best_angle = 0.0;
        double min_cost = 1e9;
        bool valid_gap_found = false;

        // [NEW] 조향각 한계 상수 (약 26도 ~ 30도)
        const double MAX_STEER_ANGLE = 0.5; 

        if (has_global_path_) {
            double global_desired_angle = get_global_goal_angle(dyn_lookahead);
            for (const auto& gap : gaps) {
                // [NEW] 조향각 한계 필터: 물리적으로 꺾을 수 없는 각도는 무시
                if (std::abs(gap.center_angle) > MAX_STEER_ANGLE) continue;

                double angle_diff = std::abs(gap.center_angle - global_desired_angle);
                double width_score = gap.len * 0.1; 
                double total_cost = angle_diff - width_score;
                
                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_angle = gap.center_angle;
                    valid_gap_found = true;
                }
            }
        } else {
            int max_len = -1;
            for (const auto& gap : gaps) {
                // [NEW] 조향각 한계 필터
                if (std::abs(gap.center_angle) > MAX_STEER_ANGLE) continue;

                if (gap.len > max_len) {
                    max_len = gap.len;
                    best_angle = gap.center_angle;
                    valid_gap_found = true;
                }
            }
        }

        // 틈은 있지만, 내 조향각으로 갈 수 있는 틈이 없는 경우
        if (!valid_gap_found) {
            publish_local_path_as_global(0.0, dyn_plan_dist, 0.0); // 정지
            return;
        }

        // [NEW] 경로 유효성 검사 (Path Validation)
        // 최종 결정된 각도 방향에 장애물이 있는지 한 번 더 확인 (센서 노이즈 대비)
        int check_idx = static_cast<int>((best_angle - msg->angle_min) / msg->angle_increment);
        if (check_idx >= 0 && check_idx < (int)ranges.size()) {
            // 내가 가려는 거리(dyn_plan_dist)보다 장애물이 더 가까이 있다면?
            // 단, ranges 값이 0.0(필터됨)인 경우는 제외하고 실제 거리값일 때만 체크
            if (ranges[check_idx] > 0.05 && ranges[check_idx] < dyn_plan_dist) {
                // 충돌 위험! 속도 0으로 설정하여 급정거 유도
                target_speed = 0.0;
            }
        }

        // 6. 경로 발행
        publish_local_path_as_global(best_angle, dyn_plan_dist, target_speed);
    }

    double get_global_goal_angle(double lookahead_dist) {
        if (global_path_.poses.empty()) return 0.0;
        try {
            if (!tf_buffer_->canTransform("map", "ego_racecar/base_link", rclcpp::Time(0))) return 0.0;
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "ego_racecar/base_link", rclcpp::Time(0));
            
            double my_x = t.transform.translation.x;
            double my_y = t.transform.translation.y;

            double min_dist_sq = 1e9;
            size_t best_idx = 0;
            for (size_t i = 0; i < global_path_.poses.size(); ++i) {
                double dx = global_path_.poses[i].pose.position.x - my_x;
                double dy = global_path_.poses[i].pose.position.y - my_y;
                double d_sq = dx*dx + dy*dy;
                if (d_sq < min_dist_sq) { min_dist_sq = d_sq; best_idx = i; }
            }

            size_t target_idx = best_idx;
            double dist_sum = 0.0;
            for (size_t i = best_idx; i < global_path_.poses.size() - 1; ++i) {
                double dx = global_path_.poses[i+1].pose.position.x - global_path_.poses[i].pose.position.x;
                double dy = global_path_.poses[i+1].pose.position.y - global_path_.poses[i].pose.position.y;
                dist_sum += std::hypot(dx, dy);
                if (dist_sum >= lookahead_dist) { target_idx = i + 1; break; }
            }

            double gx = global_path_.poses[target_idx].pose.position.x;
            double gy = global_path_.poses[target_idx].pose.position.y;

            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, car_yaw;
            m.getRPY(roll, pitch, car_yaw);

            double global_angle = std::atan2(gy - my_y, gx - my_x);
            double relative_angle = global_angle - car_yaw;
            while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
            while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;
            return relative_angle;
        } catch (tf2::TransformException &ex) { return 0.0; }
    }

    void apply_bubble(vector<double>& ranges, double angle_increment) {
        int closest_idx = -1;
        double min_dist = 100.0;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0.0 && ranges[i] < min_dist) {
                min_dist = ranges[i];
                closest_idx = i;
            }
        }
        if (closest_idx != -1) {
            // [NEW] 동적 버블 적용
            // 속도가 빠르면 버블을 키움: 기본 + (속도 * 0.1)
            // 예: 5m/s -> 0.45 + 0.5 = 0.95m (상당히 커짐)
            double dynamic_radius = bubble_radius_ + (std::abs(current_speed_) * 0.1);

            int radius_idx = static_cast<int>(dynamic_radius / (min_dist * angle_increment));
            int start_zero = std::max(0, closest_idx - radius_idx);
            int end_zero = std::min((int)ranges.size() - 1, closest_idx + radius_idx);
            for (int i = start_zero; i <= end_zero; ++i) ranges[i] = 0.0; 
        }
    }

    vector<Gap> find_gaps(const vector<double>& ranges, double angle_min, double angle_inc) {
        vector<Gap> gaps;
        int current_len = 0;
        int start_idx = 0;
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > gap_threshold_) {
                if (current_len == 0) start_idx = i;
                current_len++;
            } else {
                if (current_len > 0) {
                    Gap gap; gap.start_idx = start_idx; gap.end_idx = i - 1; gap.len = current_len;
                    gap.center_angle = angle_min + (start_idx + current_len/2) * angle_inc;
                    gaps.push_back(gap);
                }
                current_len = 0;
            }
        }
        if (current_len > 0) {
            Gap gap; gap.start_idx = start_idx; gap.end_idx = ranges.size() - 1; gap.len = current_len;
            gap.center_angle = angle_min + (start_idx + current_len/2) * angle_inc;
            gaps.push_back(gap);
        }
        return gaps;
    }

    void publish_local_path_as_global(double target_angle, double dist, double speed) {
        double local_x = dist * std::cos(target_angle);
        double local_y = dist * std::sin(target_angle);

        geometry_msgs::msg::PoseStamped local_pose, global_pose;
        local_pose.header.frame_id = "ego_racecar/base_link"; 
        local_pose.header.stamp = this->now();
        local_pose.pose.position.x = local_x;
        local_pose.pose.position.y = local_y;
        local_pose.pose.position.z = 0.0;
        
        tf2::Quaternion q; q.setRPY(0, 0, target_angle);
        local_pose.pose.orientation = tf2::toMsg(q);

        try {
            global_pose = tf_buffer_->transform(local_pose, "map", std::chrono::milliseconds(100));
        } catch (tf2::TransformException &ex) { return; }

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        geometry_msgs::msg::PoseStamped start_pose_local, start_pose_global;
        start_pose_local.header.frame_id = local_pose.header.frame_id;
        start_pose_local.header.stamp = this->now();
        start_pose_local.pose.orientation.w = 1.0;
        
        try {
            start_pose_global = tf_buffer_->transform(start_pose_local, "map", std::chrono::milliseconds(100));
            path_msg.poses.push_back(start_pose_global);
        } catch (...) {}

        global_pose.pose.position.z = speed; 
        path_msg.poses.push_back(global_pose);

        path_pub_->publish(path_msg);
    }

    vector<double> clean_ranges(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        vector<double> clean;
        clean.reserve(msg->ranges.size());
        const double MIN_FORWARD_DIST = 0.2; 
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double r = msg->ranges[i];
            if (angle < -fov_rad_ || angle > fov_rad_) { clean.push_back(0.0); continue; }
            if (std::isinf(r) || std::isnan(r) || r > msg->range_max) { clean.push_back(msg->range_max); continue; }
            double x = r * std::cos(angle);
            if (x < MIN_FORWARD_DIST) { clean.push_back(0.0); } 
            else { clean.push_back(r); }
        }
        return clean;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FGMNode>());
    rclcpp::shutdown();
    return 0;
}