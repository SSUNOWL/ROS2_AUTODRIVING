#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "mpc_controller/mpc_solver.hpp"

using namespace std::chrono_literals;

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        this->declare_parameter("horizon", 10);
        this->declare_parameter("dt", 0.05);    
        // target_speed는 Frenet Planner가 속도를 주지 않을 때의 예비값으로 사용
        this->declare_parameter("target_speed", 3.0);

        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        solver_.init(horizon, dt);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        
        // [수정] Frenet Planner의 로컬 경로 구독
        sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/fgm_path", 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        pub_predicted_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10);
        
        // MPC가 실제로 추종하려고 변환한 로컬 경로를 시각화 (디버깅용)
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
    nav_msgs::msg::Path latest_ref_path_; // Frenet Planner에서 받은 경로 저장
    
    bool odom_received_ = false;
    double target_speed_ = 2.0;

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

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        latest_ref_path_ = *msg;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void controlLoop()
    {
        if (!odom_received_) return;
        if (latest_ref_path_.poses.empty()) return;
        if (!std::isfinite(current_state_.x)) return;

        // -----------------------------------------------------------
        // [Preprocessing] Frenet Path를 Local MPC용 포맷으로 변환
        // -----------------------------------------------------------
        std::vector<mpc_controller::State> local_ref_traj;
        int horizon = this->get_parameter("horizon").as_int();

        double cx = current_state_.x;
        double cy = current_state_.y;
        double cpsi = current_state_.psi;
        double cos_psi = std::cos(cpsi);
        double sin_psi = std::sin(cpsi);

        // Frenet Planner는 'map' 프레임으로 발행한다고 가정
        // (Frenet 코드에서 path_msg.header.frame_id = "map"; 확인됨)
        
        // 1. 가장 가까운 점 찾기
        int closest_idx = -1;
        double min_d = 1e9;
        for(size_t i=0; i<latest_ref_path_.poses.size(); ++i) {
            double dx = latest_ref_path_.poses[i].pose.position.x - cx;
            double dy = latest_ref_path_.poses[i].pose.position.y - cy;
            double d = dx*dx + dy*dy;
            if(d < min_d) { min_d = d; closest_idx = i; }
        }

        if(closest_idx == -1) return;

        // 2. Horizon만큼 잘라서 변환 및 속도(z) 추출
        int search_idx = closest_idx;
        for(int i=0; i<horizon; ++i) {
            if(search_idx >= (int)latest_ref_path_.poses.size()) break;
            
            auto pt = latest_ref_path_.poses[search_idx].pose.position;
            
            // Global -> Local 위치 변환
            double dx = pt.x - cx;
            double dy = pt.y - cy;

            mpc_controller::State local_pt;
            local_pt.x = dx * cos_psi + dy * sin_psi;
            local_pt.y = -dx * sin_psi + dy * cos_psi;
            local_pt.psi = 0.0; // Yaw는 근사 (Frenet Path가 부드럽다면 0으로 둬도 무방)
            
            // [중요 수정] z축 값을 속도로 사용!
            double ref_v = pt.z; 
            
            // 속도가 너무 작거나 비정상적이면 기본값 사용 (안전장치)
            if (ref_v < 0.1 || !std::isfinite(ref_v)) {
                ref_v = target_speed_;
            }
            local_pt.vx = ref_v; 
            
            local_pt.vy = 0; 
            local_pt.omega = 0;

            local_ref_traj.push_back(local_pt);
            search_idx++; 
        }

        if (local_ref_traj.empty()) return;

        // -----------------------------------------------------------
        // [Control] MPC Solver 실행 (장애물 회피 로직 없음)
        // -----------------------------------------------------------
        mpc_controller::State local_current_state;
        local_current_state.x = 0.0;
        local_current_state.y = 0.0;
        local_current_state.psi = 0.0;
        local_current_state.vx = current_state_.vx;
        local_current_state.vy = current_state_.vy;
        local_current_state.omega = current_state_.omega;

        // 장애물 정보 없이 순수 경로 추종
        mpc_controller::Input input = solver_.solve(local_current_state, local_ref_traj);

        // -----------------------------------------------------------
        // [Actuation] Drive Command
        // -----------------------------------------------------------
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "ego_racecar/base_link";

        drive_msg.drive.steering_angle = input.delta; 
        drive_msg.drive.acceleration = input.acc;

        // [Dynamic Speed Control]
        // Frenet Planner가 이미 코너링과 장애물을 고려해 속도(z)를 줄여서 줬겠지만,
        // MPC의 조향각을 기반으로 한 번 더 안전장치를 겁니다.
        double steering_ratio = std::abs(input.delta) / 0.4189; 
        if (steering_ratio > 1.0) steering_ratio = 1.0; 
        
        // Frenet이 준 속도(ref_v)와 MPC가 최적화한 속도, 그리고 조향각 기반 감속 중 
        // 가장 보수적인 것을 선택하는 것이 안전합니다.
        // 하지만 여기서는 간단히 '설정된 타겟 속도' 기반으로 감속 비율만 적용합니다.
        // (Frenet이 준 속도를 우선시하려면 로직 수정 가능)
        
        // 여기서는 Frenet Path의 첫 번째 점 속도를 기준 속도로 삼고 감속을 적용해 보겠습니다.
        double base_speed = local_ref_traj.front().vx;
        double dynamic_speed = base_speed * (1.0 - 0.5 * steering_ratio); 
        dynamic_speed = std::max(dynamic_speed, 1.0); // 최소 1.0m/s

        drive_msg.drive.speed = dynamic_speed;
        pub_drive_->publish(drive_msg);

        // -----------------------------------------------------------
        // [Visualization]
        // -----------------------------------------------------------
        // 1. MPC 예측 경로 (초록색)
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

        // 2. MPC가 추종 중인 Local Reference (빨간색 - 디버깅용)
        // 변환된 경로가 제대로 들어왔는지, 속도(z)는 맞는지 확인 가능
        nav_msgs::msg::Path ref_viz_msg;
        ref_viz_msg.header.stamp = this->now();
        ref_viz_msg.header.frame_id = "ego_racecar/base_link";
        for(const auto& ls : local_ref_traj) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = ls.x;
            p.pose.position.y = ls.y;
            p.pose.position.z = ls.vx; // 시각화 시 z축 높이로 속도 확인 가능
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