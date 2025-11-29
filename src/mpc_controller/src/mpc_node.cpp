#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" // 경로 시각화용
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
        this->declare_parameter("horizon", 20);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("target_speed", 2.0);

        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        solver_.init(horizon, dt);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // [추가됨] 예측 경로 시각화 Publisher (Rviz용)
        pub_predicted_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10);
        
        // [추가됨] 참조 경로(Local) 시각화 Publisher
        pub_ref_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "/mpc_reference_path", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&MPCNode::controlLoop, this));
        
    }

private:
    mpc_controller::MPCSolver solver_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    
    // 시각화용 퍼블리셔
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_predicted_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;

    rclcpp::TimerBase::SharedPtr timer_;

    mpc_controller::State current_state_;
    std::vector<mpc_controller::State> global_path_;
    
    
    bool odom_received_ = false;
    bool path_received_ = false;
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

        // [복구됨] 무게중심(CoM) 보정
        // 이걸 켜야 코너링 시 "안쪽을 파고드는" 날카로운 주행이 가능합니다.
        double L_r = 0.17145; 
        current_state_.x = rear_x + L_r * std::cos(yaw);
        current_state_.y = rear_y + L_r * std::sin(yaw);
        current_state_.psi = yaw;

        double v_x_rear = msg->twist.twist.linear.x;
        double v_y_rear = msg->twist.twist.linear.y; // 뒷바퀴 기준 횡속도
        double omega = msg->twist.twist.angular.z;
        // 속도 정보
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

    // [중요] 각도 불연속성 해결 함수 (-PI ~ PI 점프 방지)
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void controlLoop()
    {
        if (!odom_received_ || !path_received_ || global_path_.empty()) return;

        if (!std::isfinite(current_state_.x) || !std::isfinite(current_state_.vx)) {
            return;
        }
        // 1. 가장 가까운 점 찾기
        int closest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_path_.size(); ++i) {
            double dx = global_path_[i].x - current_state_.x;
            double dy = global_path_[i].y - current_state_.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < min_dist) { min_dist = dist; closest_idx = i; }
        }
        if (closest_idx == -1) return;

        // 2. Reference Trajectory 생성 (Local 좌표계로 변환!)
        std::vector<mpc_controller::State> local_ref_traj; // 로컬 경로 저장용
        
        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        double current_speed = std::max(current_state_.vx, 1.0);
        
        // 내 차의 현재 자세 (좌표 변환용)
        double cx = current_state_.x;
        double cy = current_state_.y;
        double cpsi = current_state_.psi;
        double cos_psi = std::cos(cpsi);
        double sin_psi = std::sin(cpsi);

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
            
            // [핵심] Global -> Local(Body) 좌표 변환
            // 내 차가 원점(0,0)이고 앞을 보는(0도) 세상으로 변환
            double dx = global_pt.x - cx;
            double dy = global_pt.y - cy;

            mpc_controller::State local_pt;
            local_pt.x = dx * cos_psi + dy * sin_psi;   // 회전 변환
            local_pt.y = -dx * sin_psi + dy * cos_psi;
            
            // 각도 변환 (내 차 기준 상대 각도)
            double diff_psi = global_pt.psi - cpsi;
            while (diff_psi > M_PI) diff_psi -= 2.0 * M_PI;
            while (diff_psi < -M_PI) diff_psi += 2.0 * M_PI;
            local_pt.psi = diff_psi;

            // 속도 등 나머지는 그대로
            local_pt.vx = global_pt.vx; // 혹은 target_speed_
            if (std::abs(local_pt.vx) < 0.1) local_pt.vx = target_speed_;
            local_pt.vy = 0.0;
            local_pt.omega = 0.0;

            local_ref_traj.push_back(local_pt);
        }

        // 3. MPC Solver 실행 (Local Frame 기준)
        // 현재 상태는 항상 (0, 0, 0, v, 0, w) 입니다!
        mpc_controller::State local_current_state;
        local_current_state.x = 0.0;
        local_current_state.y = 0.0;
        local_current_state.psi = 0.0; // 중요! 내 차는 항상 0도를 봄
        local_current_state.vx = current_state_.vx;
        local_current_state.vy = current_state_.vy;
        local_current_state.omega = current_state_.omega;

        mpc_controller::Input input = solver_.solve(local_current_state, local_ref_traj);

        // 4. Drive Command 발행
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";
        
        // Body Frame에서는 조향각 부호 이슈가 명확해집니다.
        // 왼쪽(Local y+)으로 가려면 양수, 오른쪽이면 음수.
        // 시뮬레이터에 맞춰 부호를 결정하세요. (일단 - 유지)
        drive_msg.drive.steering_angle = input.delta; 
        drive_msg.drive.acceleration = input.acc;

        // 속도 명령
        auto local_pred_traj = solver_.getPredictedTrajectory();
        double cmd_speed = (local_pred_traj.size() > 1) ? local_pred_traj[1].vx : target_speed_;
        if (std::abs(cmd_speed) > 0.1 && std::abs(cmd_speed) < 1.0) {
            cmd_speed = (cmd_speed > 0) ? 1.0 : -1.0;
        }
        drive_msg.drive.speed = cmd_speed;
        pub_drive_->publish(drive_msg);

        // 5. 시각화 (Local 예측 경로를 다시 Global로 변환해서 보여줌)
        nav_msgs::msg::Path pred_path_msg;
        pred_path_msg.header.stamp = this->now();
        pred_path_msg.header.frame_id = "map"; // Map 기준

        for(const auto& ls : local_pred_traj) {
            geometry_msgs::msg::PoseStamped p;
            // Local -> Global 역변환
            // x_global = x_local * cos - y_local * sin + cx
            // y_global = x_local * sin + y_local * cos + cy
            p.pose.position.x = ls.x * cos_psi - ls.y * sin_psi + cx;
            p.pose.position.y = ls.x * sin_psi + ls.y * cos_psi + cy;
            pred_path_msg.poses.push_back(p);
        }
        pub_predicted_path_->publish(pred_path_msg);

        // Reference Path (얘는 이미 Global 좌표였던걸 가져와서 뿌림)
        nav_msgs::msg::Path ref_path_msg;
        ref_path_msg.header.stamp = this->now();
        ref_path_msg.header.frame_id = "map";
        // 원본 global_path에서 가져온 점들을 다시 찾기 번거로우니
        // local_ref_traj를 역변환해서 뿌려줍니다 (디버깅용으로 더 좋음)
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