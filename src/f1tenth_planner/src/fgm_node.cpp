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

        // --- [3] 속도 제어 파라미터 (NEW) ---
        this->declare_parameter("max_speed", 6.0);       // 장애물 없을 때 최대 속도
        this->declare_parameter("min_speed", 2.5);       // 장애물 앞 최소 속도
        this->declare_parameter("slow_down_dist", 4.0);  // 감속을 시작할 거리 (m)

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

        // [NEW] 속도 파라미터 읽기
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

        RCLCPP_INFO(this->get_logger(), "Adaptive Speed FGM Started. Max: %.1f, Min: %.1f", max_speed_, min_speed_);
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
    
    // 속도 관련 변수
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
            // 0.0은 side filter나 fov로 걸러진 값이므로 무시
            if (r > 0.05 && r < min_obs_dist) {
                min_obs_dist = r;
            }
        }

        // 3. 목표 속도 계산 (Adaptive Speed Logic)
        double target_speed = max_speed_;
        if (min_obs_dist < slow_down_dist_) {
            // 거리가 가까울수록 속도를 줄임 (선형 보간)
            // 거리: bubble_radius ~ slow_down_dist  ==> 속도: min_speed ~ max_speed
            double ratio = (min_obs_dist - bubble_radius_) / (slow_down_dist_ - bubble_radius_);
            ratio = std::clamp(ratio, 0.0, 1.0); // 0~1 사이로 제한
            target_speed = min_speed_ + (max_speed_ - min_speed_) * ratio;
        }

        // 4. 버블 및 Gap 찾기
        apply_bubble(ranges, msg->angle_increment);
        vector<Gap> gaps = find_gaps(ranges, msg->angle_min, msg->angle_increment);
        
        // 동적 파라미터 계산
        double dyn_plan_dist = std::clamp(min_plan_dist_ + plan_gain_ * current_speed_, min_plan_dist_, max_plan_dist_);
        double dyn_lookahead = std::clamp(min_look_ + look_gain_ * current_speed_, min_look_, max_look_);

        if (gaps.empty()) {
            // 비상시: 최저 속도로 직진 시도
            publish_local_path_as_global(0.0, dyn_plan_dist, min_speed_);
            return;
        }

        // 5. 최적 각도 선정
        double best_angle = 0.0;
        double min_cost = 1e9;

        if (has_global_path_) {
            double global_desired_angle = get_global_goal_angle(dyn_lookahead);
            for (const auto& gap : gaps) {
                double angle_diff = std::abs(gap.center_angle - global_desired_angle);
                double width_score = gap.len * 0.1; 
                double total_cost = angle_diff - width_score;
                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_angle = gap.center_angle;
                }
            }
        } else {
            int max_len = -1;
            for (const auto& gap : gaps) {
                if (gap.len > max_len) {
                    max_len = gap.len;
                    best_angle = gap.center_angle;
                }
            }
        }

        // 6. 경로 발행 (계산된 target_speed 전달)
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
            if (ranges[i] > 0.0 && ranges[i] < min_dist) { // 0.0 제외
                min_dist = ranges[i];
                closest_idx = i;
            }
        }
        if (closest_idx != -1) {
            int radius_idx = static_cast<int>(bubble_radius_ / (min_dist * angle_increment));
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

    // [수정] target_speed 인자 추가
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

        // 계산된 속도(speed)를 입력
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