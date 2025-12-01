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
#include <numeric>

using namespace std;
using namespace std::chrono_literals;

struct Gap {
    int start_idx;
    int end_idx;
    int len;
    double center_angle;
    double avg_depth; 
};

class FGMNode : public rclcpp::Node {
public:
    FGMNode() : Node("fgm_node") {
        // --- [1] 안전 및 인식 파라미터 ---
        this->declare_parameter("gap_threshold", 1.2); 
        this->declare_parameter("bubble_radius", 0.5); 
        this->declare_parameter("fov_angle", 180.0);
        
        // --- [2] 경로 생성 파라미터 ---
        this->declare_parameter("min_planning_dist", 2.0);
        this->declare_parameter("max_planning_dist", 5.0);
        this->declare_parameter("planning_gain", 1.0); 
        
        this->declare_parameter("min_lookahead", 1.5);
        this->declare_parameter("max_lookahead", 3.5);
        this->declare_parameter("lookahead_gain", 0.6);

        // --- [3] 속도 제어 파라미터 ---
        this->declare_parameter("max_speed", 6.0);       
        this->declare_parameter("min_speed", 2.0);       
        this->declare_parameter("slow_down_dist", 2.5);

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

        RCLCPP_INFO(this->get_logger(), "FGM Curve Mode Started.");
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
    
    double last_best_angle_ = 0.0; 

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
        vector<double> ranges = clean_ranges(msg);

        // 2. 속도 제어 (이전과 동일)
        double front_min_dist = 100.0;
        double speed_check_fov = 30.0 * (M_PI / 180.0);
        for (size_t i = 0; i < ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::abs(angle) < speed_check_fov) { 
                if (ranges[i] > 0.05 && ranges[i] < front_min_dist) front_min_dist = ranges[i];
            }
        }
        double target_speed = max_speed_;
        if (front_min_dist < slow_down_dist_) {
            double ratio = (front_min_dist - bubble_radius_) / (slow_down_dist_ - bubble_radius_);
            ratio = std::clamp(ratio, 0.0, 1.0);
            target_speed = min_speed_ + (max_speed_ - min_speed_) * (ratio * ratio);
        }

        // 3. Gap 찾기
        apply_bubble(ranges, msg->angle_increment);
        vector<Gap> gaps = find_gaps(ranges, msg->angle_min, msg->angle_increment);
        
        double dyn_plan_dist = std::clamp(min_plan_dist_ + plan_gain_ * current_speed_, min_plan_dist_, max_plan_dist_);
        double dyn_lookahead = std::clamp(min_look_ + look_gain_ * current_speed_, min_look_, max_look_);

        if (gaps.empty()) {
            publish_curved_path(last_best_angle_ * 0.5, 1.5, 1.0); 
            return;
        }

        // 4. 최적 Gap 선정 (Wall Margin 적용)
        double best_angle = 0.0;
        double max_score = -1e9;
        bool valid_gap_found = false;

        double global_desired_angle = 0.0;
        if (has_global_path_) {
            global_desired_angle = get_global_goal_angle(dyn_lookahead);
        }

        const double HYSTERESIS_BONUS = 2.0; 
        const double CHANGE_THRESHOLD = 0.3; 
        
        // [NEW] 벽 이격 마진 (라디안 단위)
        // 약 0.25 rad = 14.3도. 이 값만큼 벽에서 떨어집니다.
        // 차폭을 고려하여 충분히 줘야 합니다.
        const double WALL_MARGIN_ANGLE = 0.3; 

        for (const auto& gap : gaps) {
            // Gap의 실제 물리적 시작/끝 각도
            double gap_start_angle = msg->angle_min + gap.start_idx * msg->angle_increment;
            double gap_end_angle = msg->angle_min + gap.end_idx * msg->angle_increment;

            // [핵심] 안전 구간 계산 (Gap 양 끝에서 Margin만큼 깎아냄)
            double safe_min_angle = gap_start_angle + WALL_MARGIN_ANGLE;
            double safe_max_angle = gap_end_angle - WALL_MARGIN_ANGLE;

            // 마진을 깎았더니 남는 공간이 없다면? (너무 좁은 틈) -> 건너뜀
            if (safe_min_angle >= safe_max_angle) continue;

            // 1. 일단 Gap의 중앙을 타겟으로 잡음
            double target_angle_in_gap = (safe_min_angle + safe_max_angle) / 2.0;

            // 2. Global Path 수렴 로직 (Bias)
            if (has_global_path_) {
                // Global Path가 안전 구간(Safe Zone) 안에 있다면?
                if (global_desired_angle >= safe_min_angle && global_desired_angle <= safe_max_angle) {
                    // 안전하므로 Global Path 방향으로 강하게 당김 (80% 반영)
                    target_angle_in_gap = (target_angle_in_gap * 0.2) + (global_desired_angle * 0.8);
                }
                else if (global_desired_angle < safe_min_angle) {
                    // Global Path가 오른쪽 벽 너머에 있음 -> 안전 구간의 가장 오른쪽 끝 선택
                    target_angle_in_gap = safe_min_angle; 
                }
                else { // global_desired_angle > safe_max_angle
                    // Global Path가 왼쪽 벽 너머에 있음 -> 안전 구간의 가장 왼쪽 끝 선택
                    target_angle_in_gap = safe_max_angle;
                }
            }

            // 점수 계산
            double width_score = gap.len; 
            double angle_diff = std::abs(target_angle_in_gap - global_desired_angle);
            double steer_penalty = std::abs(target_angle_in_gap) * 5.0;

            double score = (width_score * 0.5) - (angle_diff * 4.0) - (steer_penalty * 0.1);

            if (std::abs(target_angle_in_gap - last_best_angle_) < CHANGE_THRESHOLD) {
                score += HYSTERESIS_BONUS;
            }

            if (score > max_score) {
                max_score = score;
                best_angle = target_angle_in_gap;
                valid_gap_found = true;
            }
        }

        if (!valid_gap_found) {
             // 마진을 적용했더니 갈 곳이 하나도 없다면? 
             // 비상 로직: 마진 무시하고 가장 넓은 곳 중앙으로 (긁으면서라도 가야함)
             int max_len = -1;
             for(auto &g : gaps) {
                 if(g.len > max_len) {
                     max_len = g.len;
                     best_angle = g.center_angle;
                 }
             }
        }

        // 5. 조향각 평활화
        double alpha = 0.4; 
        best_angle = (alpha * best_angle) + ((1.0 - alpha) * last_best_angle_);
        last_best_angle_ = best_angle;

        // 6. 충돌 방지 및 경로 발행 (동일)
        int check_idx = static_cast<int>((best_angle - msg->angle_min) / msg->angle_increment);
        int window = 3;
        double sum_dist = 0; int cnt = 0;
        for(int k=-window; k<=window; ++k) {
            int idx = check_idx + k;
            if(idx >= 0 && idx < (int)ranges.size()) {
                if(ranges[idx] > 0.1) { sum_dist += ranges[idx]; cnt++; }
            }
        }
        double avg_dist = (cnt > 0) ? sum_dist/cnt : 0.0;
        if (cnt > 0 && avg_dist < dyn_plan_dist * 0.7) {
             target_speed *= 0.5; dyn_plan_dist = avg_dist * 0.8;
        }

        publish_curved_path(best_angle, dyn_plan_dist, target_speed);
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
            if (ranges[i] > 0.05 && ranges[i] < min_dist) {
                min_dist = ranges[i];
                closest_idx = i;
            }
        }

        if (closest_idx != -1) {
            double dynamic_radius = bubble_radius_ + (std::abs(current_speed_) * 0.1); 
            int radius_idx = static_cast<int>(dynamic_radius / (min_dist * angle_increment));
            int start_idx = std::max(0, closest_idx - radius_idx);
            int end_idx = std::min((int)ranges.size() - 1, closest_idx + radius_idx);
            
            for (int i = start_idx; i <= end_idx; ++i) {
                ranges[i] = 0.0; 
            }
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
                if (current_len > 5) { 
                    Gap gap; 
                    gap.start_idx = start_idx; 
                    gap.end_idx = i - 1; 
                    gap.len = current_len;
                    gap.center_angle = angle_min + (start_idx + current_len/2) * angle_inc;
                    
                    double sum_depth = 0;
                    for(int k=start_idx; k<i; k++) sum_depth += ranges[k];
                    gap.avg_depth = sum_depth / current_len;
                    
                    gaps.push_back(gap);
                }
                current_len = 0;
            }
        }
        if (current_len > 5) {
            Gap gap; 
            gap.start_idx = start_idx; 
            gap.end_idx = ranges.size() - 1; 
            gap.len = current_len;
            gap.center_angle = angle_min + (start_idx + current_len/2) * angle_inc;
            
            double sum_depth = 0;
            for(size_t k=start_idx; k<ranges.size(); k++) sum_depth += ranges[k];
            gap.avg_depth = sum_depth / current_len;

            gaps.push_back(gap);
        }
        return gaps;
    }

    // [NEW] 곡선 경로 발행 함수 (직선 2점 -> 곡선 20점)
    void publish_curved_path(double target_angle, double dist, double speed) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map"; 

        // 1. 현재 차량 위치 및 Yaw 구하기
        geometry_msgs::msg::TransformStamped t;
        try {
            if (!tf_buffer_->canTransform("map", "ego_racecar/base_link", rclcpp::Time(0))) return;
            t = tf_buffer_->lookupTransform("map", "ego_racecar/base_link", rclcpp::Time(0));
        } catch (tf2::TransformException &ex) { return; }

        double car_x = t.transform.translation.x;
        double car_y = t.transform.translation.y;
        
        tf2::Quaternion q_car(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        double roll, pitch, car_yaw;
        tf2::Matrix3x3(q_car).getRPY(roll, pitch, car_yaw);

        double local_goal_x = dist * std::cos(target_angle);
        double local_goal_y = dist * std::sin(target_angle);

        // 2. 곡선 보간 (Interpolation) - 20개의 점 생성
        int num_points = 20;
        for (int i = 0; i <= num_points; ++i) {
            double t_ratio = (double)i / num_points;
            
            double current_dist = dist * t_ratio;
            double current_local_angle = target_angle * t_ratio; // 각도를 선형적으로 변화시킴

            // 로컬 곡선 좌표 (회전 성분 + 직선 성분 혼합)
            double lx = current_dist * std::cos(current_local_angle);
            double ly = current_dist * std::sin(current_local_angle);
            
            // 끝점이 목표점에 정확히 맞도록 직선 보간 성분 섞기 (7:3 비율)
            double straight_lx = local_goal_x * t_ratio;
            double straight_ly = local_goal_y * t_ratio;
            
            lx = lx * 0.7 + straight_lx * 0.3;
            ly = ly * 0.7 + straight_ly * 0.3;

            // 로컬 -> 글로벌 좌표 변환
            double gx = car_x + (lx * std::cos(car_yaw) - ly * std::sin(car_yaw));
            double gy = car_y + (lx * std::sin(car_yaw) + ly * std::cos(car_yaw));

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = gx;
            pose.pose.position.y = gy;
            pose.pose.position.z = speed; 

            // 경로의 헤딩 계산 (다음 점을 바라보게)
            double global_heading = car_yaw + current_local_angle;
            tf2::Quaternion q;
            q.setRPY(0, 0, global_heading);
            pose.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    vector<double> clean_ranges(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        vector<double> clean;
        clean.reserve(msg->ranges.size());
        const double MIN_FORWARD_DIST = 0.1; 
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double r = msg->ranges[i];
            
            if (angle < -fov_rad_ || angle > fov_rad_) { 
                clean.push_back(0.0); 
                continue; 
            }
            if (std::isinf(r) || std::isnan(r) || r > msg->range_max) { 
                clean.push_back(msg->range_max); 
                continue; 
            }
            if (r < MIN_FORWARD_DIST) { 
                clean.push_back(0.0); 
            } else { 
                clean.push_back(r); 
            }
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