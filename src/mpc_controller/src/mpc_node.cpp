#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"             
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "mpc_controller/mpc_solver.hpp"

using namespace std::chrono_literals;

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        this->declare_parameter("horizon", 20);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("target_speed", 2.0);

        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        solver_.init(horizon, dt);

        // 1. Odom & Path 구독
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // 2. TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. LiDAR
        auto qos = rclcpp::SensorDataQoS();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&MPCNode::scan_callback, this, std::placeholders::_1));

        // 4. 시각화 퍼블리셔
        pub_predicted_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10);
        
        pub_ref_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_reference_path", 10);

        pub_obstacles_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mpc_obstacles_viz", 10);

        // 5. 타이머
        timer_ = this->create_wall_timer(
            50ms, std::bind(&MPCNode::controlLoop, this));
    }

private:
    mpc_controller::MPCSolver solver_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_predicted_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_viz_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    mpc_controller::State current_state_;
    std::vector<mpc_controller::State> global_path_;
    std::vector<mpc_controller::MPCSolver::Obstacle> obstacles_;

    bool odom_received_ = false;
    bool path_received_ = false;
    double target_speed_ = 2.0;

    // [추가] 회피 상태 관리를 위한 변수
    int avoid_direction_ = 0; // 0: 없음, 1: 왼쪽(y+), -1: 오른쪽(y-)
    int avoid_counter_ = 0;   // 상태 유지 카운터

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double rear_x = msg->pose.pose.position.x;
        double rear_y = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double L_r = 0.17145; 
        current_state_.x = rear_x + L_r * std::cos(yaw);
        current_state_.y = rear_y + L_r * std::sin(yaw);
        current_state_.psi = yaw;

        double v_x_rear = msg->twist.twist.linear.x;
        double v_y_rear = msg->twist.twist.linear.y; 
        double omega = msg->twist.twist.angular.z;
        
        current_state_.vx = v_x_rear; 
        current_state_.vy = v_y_rear + omega * L_r; 
        current_state_.omega = omega;

        odom_received_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_.clear();
        for (const auto& pose : msg->poses) {
            mpc_controller::State s;
            s.x = pose.pose.position.x;
            s.y = pose.pose.position.y;
            
            tf2::Quaternion q(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            s.psi = y;
            s.vx = target_speed_; 
            global_path_.push_back(s);
        }
        path_received_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacles_.clear(); 

        std::string target_frame = "map"; 
        std::string source_frame = msg->header.frame_id; 

        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            int step = 10; 
            
            for (size_t i = 0; i < msg->ranges.size(); i += step) {
                float range = msg->ranges[i];

                if (range < msg->range_min || range > msg->range_max || range > 5.0) {
                    continue;
                }

                float angle = msg->angle_min + i * msg->angle_increment;
                float lx = range * cos(angle);
                float ly = range * sin(angle);

                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.frame_id = source_frame;
                point_in.header.stamp = msg->header.stamp;
                point_in.point.x = lx;
                point_in.point.y = ly;
                point_in.point.z = 0.0;

                tf2::doTransform(point_in, point_out, transform);

                mpc_controller::MPCSolver::Obstacle obs;
                obs.x = point_out.point.x;
                obs.y = point_out.point.y;
                obs.r = 0.2; // F1TENTH 안전 회피 반지름
                
                obstacles_.push_back(obs);
            }

            publish_obstacle_markers();

        } catch (tf2::TransformException &ex) {
            // TF 에러 무시
        }
    }

    void publish_obstacle_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        int id = 0;
        for (const auto& obs : obstacles_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = ++id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = obs.x;
            marker.pose.position.y = obs.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = obs.r * 2.0; 
            marker.scale.y = obs.r * 2.0;
            marker.scale.z = 0.1;

            marker.color.r = 1.0; 
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6; 

            marker.lifetime = rclcpp::Duration::from_seconds(0.1); 
            marker_array.markers.push_back(marker);
        }
        pub_obstacles_viz_->publish(marker_array);
    }

    void controlLoop()
    {
        if (!odom_received_ || !path_received_ || global_path_.empty()) return;
        if (!std::isfinite(current_state_.x) || !std::isfinite(current_state_.vx)) return;

        // 1. 가장 가까운 점 찾기 (Global)
        int closest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_path_.size(); ++i) {
            double dx = global_path_[i].x - current_state_.x;
            double dy = global_path_[i].y - current_state_.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < min_dist) { min_dist = dist; closest_idx = i; }
        }
        if (closest_idx == -1) return;

        double cx = current_state_.x;
        double cy = current_state_.y;
        double cpsi = current_state_.psi;
        double cos_psi = std::cos(cpsi);
        double sin_psi = std::sin(cpsi);

        // =========================================================
        // [수정 1] 장애물 인식 범위 수정 (내 차 인식 방지)
        // =========================================================
        
        bool obstacle_detected = false;
        double obs_y_min = 0.0; 
        double obs_x_min = 100.0; 
        
        double debug_detect_x = 0.0; // 디버깅용
        double debug_detect_y = 0.0;

        for (const auto& obs_global : obstacles_) {
            double dx = obs_global.x - cx;
            double dy = obs_global.y - cy;
            
            // Global -> Local 변환
            double lx = dx * cos_psi + dy * sin_psi;
            double ly = -dx * sin_psi + dy * cos_psi;
            
            // [핵심] 감지 필터 강화
            // 1. lx > 0.5: 내 차 앞 범퍼(약 0.3~0.4m) 인식 방지 (중요!)
            // 2. lx < 3.0: 너무 먼 장애물은 무시 (반응성 향상)
            // 3. abs(ly) < 0.6: 내 주행로를 막는 녀석만 봄
            if (lx > 0.5 && lx < 3.0 && std::abs(ly) < 0.6) {
                if (lx < obs_x_min) {
                    obs_x_min = lx;
                    obs_y_min = ly;
                    obstacle_detected = true;
                    debug_detect_x = lx;
                    debug_detect_y = ly;
                }
            }
        }

        // 2. 상태 머신 (회피 -> 유지 -> 복귀)
        if (obstacle_detected) {
            if (avoid_direction_ == 0) {
                if (obs_y_min > 0) avoid_direction_ = -1; // 장애물 왼쪽 -> 우측 회피
                else avoid_direction_ = 1;            // 장애물 오른쪽 -> 좌측 회피
            }
            avoid_counter_ = 3; 
        } 
        else {
            if (avoid_counter_ > 0) {
                avoid_counter_--; 
            } else {
                avoid_direction_ = 0; // 복귀
            }
        }

        // 3. Lateral Shift
        double target_shift = 0.0;
        if (avoid_direction_ == 1) target_shift = 0.5;  
        else if (avoid_direction_ == -1) target_shift = -0.5;

        static double current_shift = 0.0;
        double alpha = 0.15;
        
        current_shift = current_shift * (1.0 - alpha) + target_shift * alpha;
        if (std::abs(current_shift) < 0.05) current_shift = 0.0;

        // [디버깅 로그]
        if (obstacle_detected || std::abs(current_shift) > 0.1) {
            RCLCPP_INFO(this->get_logger(), 
                "AVOID: [Det: %d] [Obj: %.2f, %.2f] [Dir: %d] [Shift: %.2f]",
                obstacle_detected, debug_detect_x, debug_detect_y, avoid_direction_, current_shift);
        }

        // 4. Ref Traj 생성 및 Shift 적용
        std::vector<mpc_controller::State> local_ref_traj; 
        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        double current_speed = std::max(current_state_.vx, 1.0);

        int search_idx = closest_idx;
        double elapsed_dist = 0.0;

        for (int i = 0; i < horizon; ++i) {
            double target_dist = current_speed * dt * (i + 1);
            while (search_idx < (int)global_path_.size() - 1) {
                double dx = global_path_[search_idx+1].x - global_path_[search_idx].x;
                double dy = global_path_[search_idx+1].y - global_path_[search_idx].y;
                double dist_segment = std::sqrt(dx*dx + dy*dy);
                if (elapsed_dist + dist_segment >= target_dist) break; 
                elapsed_dist += dist_segment;
                search_idx++;
            }
            mpc_controller::State global_pt = global_path_[search_idx];
            
            double dx = global_pt.x - cx;
            double dy = global_pt.y - cy;
            mpc_controller::State local_pt;
            local_pt.x = dx * cos_psi + dy * sin_psi; 
            local_pt.y = -dx * sin_psi + dy * cos_psi;
            
            // [적용] 능동적 회피
            local_pt.y += current_shift; 

            double diff_psi = global_pt.psi - cpsi;
            while (diff_psi > M_PI) diff_psi -= 2.0 * M_PI;
            while (diff_psi < -M_PI) diff_psi += 2.0 * M_PI;
            local_pt.psi = diff_psi;
            local_pt.vx = global_pt.vx; 
            if (std::abs(local_pt.vx) < 0.1) local_pt.vx = target_speed_;
            local_pt.vy = 0.0;
            local_pt.omega = 0.0;
            local_ref_traj.push_back(local_pt);
        }

        // 5. MPC Solver 실행
        // [수정 2] Solver 충돌 방지를 위해 빈 장애물 리스트를 전달합니다.
        // 우리는 이미 위에서 경로(Shift)를 수정했으므로, 
        // Solver에게 또 "피하라"고 강요하면 계산이 터집니다.
        std::vector<mpc_controller::MPCSolver::Obstacle> empty_obstacles; 

        mpc_controller::State local_current_state; 
        local_current_state.x = 0; local_current_state.y=0; local_current_state.psi=0;
        local_current_state.vx = current_state_.vx; 
        local_current_state.vy = current_state_.vy;
        local_current_state.omega = current_state_.omega;

        // 장애물 없이 경로 추종만 시킴 (경로 자체가 이미 피하는 경로임)
        mpc_controller::Input input = solver_.solve(local_current_state, local_ref_traj, empty_obstacles);

        // 6. Drive
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = input.delta; 
        drive_msg.drive.acceleration = input.acc;
        
        auto local_pred_traj = solver_.getPredictedTrajectory();
        double cmd_speed = (local_pred_traj.size() > 1) ? local_pred_traj[1].vx : target_speed_;
        if (std::abs(cmd_speed) > 0.1 && std::abs(cmd_speed) < 1.0) cmd_speed = (cmd_speed > 0) ? 1.0 : -1.0;
        drive_msg.drive.speed = cmd_speed;
        pub_drive_->publish(drive_msg);

        // 7. Viz
        nav_msgs::msg::Path pred_path_msg;
        pred_path_msg.header.stamp = this->now();
        pred_path_msg.header.frame_id = "map";
        for(const auto& ls : local_pred_traj) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = ls.x * cos_psi - ls.y * sin_psi + cx;
            p.pose.position.y = ls.x * sin_psi + ls.y * cos_psi + cy;
            pred_path_msg.poses.push_back(p);
        }
        pub_predicted_path_->publish(pred_path_msg);
        
        // Shift된 경로 시각화
        nav_msgs::msg::Path ref_path_msg;
        ref_path_msg.header.stamp = this->now();
        ref_path_msg.header.frame_id = "map";
        for(const auto& ls : local_ref_traj) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = ls.x * cos_psi - ls.y * sin_psi + cx;
            p.pose.position.y = ls.x * sin_psi + ls.y * cos_psi + cy;
            ref_path_msg.poses.push_back(p);
        }
        pub_ref_path_->publish(ref_path_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}