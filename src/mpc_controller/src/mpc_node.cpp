#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mpc_controller/mpc_solver.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using namespace std;

// Pure Pursuit Node에서 사용된 Waypoint 구조체와 유사하게 정의
struct Waypoint {
    double x;
    double y;
    double v; 
};

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        // [추가] 경로 선택 파라미터 (Pure Pursuit과 동일)
        this->declare_parameter("use_frenet_path", false);
        this->declare_parameter("frenet_path_topic", "/selected_path");
        this->declare_parameter("csv_path", "raceline_with_speed.csv");

        this->declare_parameter("horizon", 10);
        this->declare_parameter("dt", 0.05);    
        this->declare_parameter("target_speed", 3.0);

        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        
        // [추가] 경로 선택 파라미터 로드
        use_frenet_path_ = this->get_parameter("use_frenet_path").as_bool();
        frenet_path_topic_ = this->get_parameter("frenet_path_topic").as_string();
        string csv_path = this->get_parameter("csv_path").as_string();


        solver_.init(horizon, dt);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        
        // [변경] 경로 로딩 로직 분기
        if (use_frenet_path_) {
            // [Frenet Path] 로컬 경로 구독 (MPC가 매 순간 동적으로 참조)
            sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(
                frenet_path_topic_, 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "MPC using Frenet path from topic: %s", frenet_path_topic_.c_str());
        } else {
            // [Static Path] CSV 파일 로드 (MPC가 전체 경로 중 일부를 참조)
            if (!load_waypoints(csv_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file for MPC!");
                // CSV 로드 실패 시 강제 종료 대신, 안전을 위해 기본 속도(3.0)로만 주행 가능하게 할 수도 있음.
                // 하지만 여기서는 기존 MPC 로직의 의도를 따름.
                rclcpp::shutdown(); 
            }
            RCLCPP_INFO(this->get_logger(), "MPC using static CSV waypoints: %s", csv_path.c_str());
        }

        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        pub_predicted_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10);
        
        pub_local_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_local_ref_debug", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&MPCNode::controlLoop, this));
    }

private:
    mpc_controller::MPCSolver solver_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_ref_path_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_predicted_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;

    rclcpp::TimerBase::SharedPtr timer_;

    mpc_controller::State current_state_;
    std::vector<Waypoint> local_waypoints_; // [변경] 참조 경로 저장 (CSV 또는 Path)
    
    bool odom_received_ = false;
    double target_speed_ = 2.0;
    
    // [추가] 경로 선택 변수
    bool use_frenet_path_;
    std::string frenet_path_topic_;


    // [추가] CSV 로딩 함수 (Pure Pursuit Node에서 복사)
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
                    local_waypoints_.push_back(wp);
                } catch (...) { continue; }
            }
        }
        return !local_waypoints_.empty();
    }

    // [변경] Frenet Path 콜백
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        // Frenet Path가 들어오면 local_waypoints_를 동적으로 업데이트
        std::vector<Waypoint> new_wps;
        new_wps.reserve(msg->poses.size());

        for (const auto & ps : msg->poses) {
            Waypoint wp;
            wp.x = ps.pose.position.x;
            wp.y = ps.pose.position.y;
            wp.v = ps.pose.position.z; // 속도는 z축에 저장되었다고 가정
            new_wps.push_back(wp);
        }

        if (!new_wps.empty()) {
            local_waypoints_ = std::move(new_wps);
        }
    }

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

    void controlLoop()
    {
        if (!odom_received_) return;
        if (local_waypoints_.empty()) return; // [변경] 참조 경로 확인
        if (!std::isfinite(current_state_.x)) return;

        // -----------------------------------------------------------
        // [Preprocessing] 현재 위치에서 Horizon 만큼의 참조 궤적 추출
        // -----------------------------------------------------------
        std::vector<mpc_controller::State> mpc_ref_traj;
        int horizon = this->get_parameter("horizon").as_int();

        double cx = current_state_.x;
        double cy = current_state_.y;
        double cpsi = current_state_.psi;
        double cos_psi = std::cos(cpsi);
        double sin_psi = std::sin(cpsi);

        // 1. 가장 가까운 점 찾기 (Static Path일 경우를 고려)
        int closest_idx = -1;
        double min_d = 1e9;
        
        // [Frenet Path]는 이미 Local Path이므로 0번 인덱스부터 시작하면 되지만,
        // [Static Path]일 경우, 전체 경로에서 현재 위치를 찾아야 합니다.
        // 여기서는 두 경우 모두 안전하게 가장 가까운 점을 찾도록 구현합니다.
        for(size_t i=0; i<local_waypoints_.size(); ++i) {
            double dx = local_waypoints_[i].x - cx;
            double dy = local_waypoints_[i].y - cy;
            double d = dx*dx + dy*dy;
            if(d < min_d) { min_d = d; closest_idx = i; }
        }

        if(closest_idx == -1) return;

        // 2. Horizon만큼 잘라서 Local MPC용 포맷으로 변환
        int search_idx = closest_idx;
        for(int i=0; i<horizon; ++i) {
            int current_wp_idx = search_idx % local_waypoints_.size(); // 랩트랙 순환 고려
            
            Waypoint wp = local_waypoints_[current_wp_idx];
            
            // Global -> Local 위치 변환
            double dx = wp.x - cx;
            double dy = wp.y - cy;

            mpc_controller::State local_pt;
            local_pt.x = dx * cos_psi + dy * sin_psi;
            local_pt.y = -dx * sin_psi + dy * cos_psi;
            local_pt.psi = 0.0; // Yaw는 근사
            
            double ref_v = wp.v; 
            
            if (ref_v < 0.1 || !std::isfinite(ref_v)) {
                ref_v = target_speed_;
            }
            local_pt.vx = ref_v; 
            
            local_pt.vy = 0; 
            local_pt.omega = 0;

            mpc_ref_traj.push_back(local_pt);
            search_idx++; 
        }

        if (mpc_ref_traj.empty()) return;

        // -----------------------------------------------------------
        // [Control] MPC Solver 실행 
        // -----------------------------------------------------------
        mpc_controller::State local_current_state;
        local_current_state.x = 0.0;
        local_current_state.y = 0.0;
        local_current_state.psi = 0.0;
        local_current_state.vx = current_state_.vx;
        local_current_state.vy = current_state_.vy;
        local_current_state.omega = current_state_.omega;

        mpc_controller::Input input = solver_.solve(local_current_state, mpc_ref_traj);

        // -----------------------------------------------------------
        // [Actuation] Drive Command
        // -----------------------------------------------------------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "ego_racecar/base_link";

        drive_msg.drive.steering_angle = input.delta; 
        drive_msg.drive.acceleration = input.acc;

        // 속도 제어는 MPC의 최적화된 가속도(input.acc)와 참조 속도(mpc_ref_traj.front().vx)를 조합하여 수행.
        // 여기서는 가장 보수적인 안전장치였던 조향각 기반 감속 로직 대신
        // 참조 경로의 속도를 그대로 사용하거나, 가속도 입력을 활용해야 MPC의 의도대로 동작합니다.
        
        // MPC가 계산한 최적의 가속도를 사용하므로, 속도는 가속도 입력을 통해 제어됩니다.
        // 다만, MPC가 계산한 결과에 따라 가속/감속이 이루어질 뿐,
        // 최종 속도 명령(drive_msg.drive.speed)은 MPC 출력이 아닌 경우가 많으므로
        // 여기서는 참조 속도의 최댓값 또는 안전 속도를 사용합니다.
        
        double ref_speed = mpc_ref_traj.front().vx;
        
        // 최대 속도 제약 (옵션: MPC 내에서 이미 제약되었을 수 있음)
        // drive_msg.drive.speed = std::min(ref_speed, max_allowed_speed);

        drive_msg.drive.speed = ref_speed; // MPC가 최적화하는 것은 가속/감속이므로, 기준 속도는 참조 경로의 속도를 따릅니다.
        pub_drive_->publish(drive_msg);

        // -----------------------------------------------------------
        // [Visualization]
        // -----------------------------------------------------------
        // (Visualization 코드는 변경 없이 유지)
        auto local_pred_traj = solver_.getPredictedTrajectory();
        nav_msgs::msg::Path pred_path_msg;
        pred_path_msg.header.stamp = this->now();
        pred_path_msg.header.frame_id = "ego_racecar/base_link"; 

        for(const auto& ls : local_pred_traj) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = ls.x;
            p.pose.position.y = ls.y;
            pred_path_msg.poses.push_back(p);
        }
        pub_predicted_path_->publish(pred_path_msg);

        nav_msgs::msg::Path ref_viz_msg;
        ref_viz_msg.header.stamp = this->now();
        ref_viz_msg.header.frame_id = "ego_racecar/base_link";
        for(const auto& ls : mpc_ref_traj) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = ls.x;
            p.pose.position.y = ls.y;
            p.pose.position.z = ls.vx; 
            ref_viz_msg.poses.push_back(p);
        }
        pub_local_path_->publish(ref_viz_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}