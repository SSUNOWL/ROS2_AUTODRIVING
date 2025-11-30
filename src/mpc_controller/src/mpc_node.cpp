#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp" // 경로 시각화용
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker.hpp" // 장애물 시각화용
#include "sensor_msgs/msg/laser_scan.hpp"
#include "mpc_controller/mpc_solver.hpp"

using namespace std::chrono_literals;

class MPCNode : public rclcpp::Node
{
public:
    MPCNode() : Node("mpc_node")
    {
        this->declare_parameter("horizon", 30);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("target_speed", 3.0);

        int horizon = this->get_parameter("horizon").as_int();
        double dt = this->get_parameter("dt").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        solver_.init(horizon, dt);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&MPCNode::odomCallback, this, std::placeholders::_1));
        
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MPCNode::scanCallback, this, std::placeholders::_1));

        pub_obs_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mpc_obstacle_marker", 10);
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
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_obs_marker_;

    std::vector<mpc_controller::Obstacle> detected_obstacles_; // 감지된 장애물 저장
    

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

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. ROI 설정 (내 차선 위주)
        double roi_min_x = 0.0; 
        double roi_max_x = 3.5; // 3.5m 이내
        double roi_min_y = -0.6; 
        double roi_max_y = 0.6;

        detected_obstacles_.clear(); // 기존 장애물 비우기

        // 시각화용 변수 (가장 가까운 놈 하나만 표시)
        mpc_controller::Obstacle closest_for_viz;
        double min_dist_sq_for_viz = std::numeric_limits<double>::max();
        bool found_any = false;

        // [최적화 1] Stride 5: 5칸씩 건너뛰며 검사
        int stride = 5; 

        for (size_t i = 0; i < msg->ranges.size(); i += stride) {
            double range = msg->ranges[i];
            
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) 
                continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            
            double lx = range * std::cos(angle);
            double ly = range * std::sin(angle);

            // ROI 필터링
            if (lx > roi_min_x && lx < roi_max_x &&
                ly > roi_min_y && ly < roi_max_y) 
            {
                // [전략 변경] ROI 내의 모든 장애물을 리스트에 추가합니다.
                mpc_controller::Obstacle obs;
                obs.x = lx;
                obs.y = ly;
                obs.r = 0.55; // 장애물 반지름 (안전 마진)
                
                detected_obstacles_.push_back(obs);

                // 시각화를 위해 가장 가까운 놈 찾기
                double d2 = lx*lx + ly*ly;
                if (d2 < min_dist_sq_for_viz) {
                    min_dist_sq_for_viz = d2;
                    closest_for_viz = obs;
                    found_any = true;
                }
            }
        }

        // 시각화는 가장 위험한(가까운) 녀석 하나만 합니다.
        // (모두 다 그리려면 MarkerArray가 필요해서 코드가 복잡해짐)
        if (found_any) {
            publishObstacleMarker(closest_for_viz);
        } else {
            // 장애물이 없으면 마커를 안 보이게 치우거나 그냥 둡니다.
            deleteObstacleMarker(); 
        }


    }

    void publishObstacleMarker(const mpc_controller::Obstacle& obs) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/base_link"; // 내 차 기준
        marker.header.stamp = rclcpp::Time(0);
        marker.ns = "mpc_obstacle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = obs.x;
        marker.pose.position.y = obs.y;
        marker.pose.position.z = 0.5; // [수정] 바닥에 묻히지 않게 50cm 띄움
        
        marker.scale.x = obs.r * 2.0; 
        marker.scale.y = obs.r * 2.0;
        marker.scale.z = 0.5; // [수정] 높이도 좀 키움

        marker.color.a = 1.0; // [수정] 투명도 1.0 (완전 불투명)
        marker.color.r = 1.0; 
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // [중요] Lifetime을 0.2초 정도로 줍니다. 
        // 0이면 영구 지속인데, 업데이트가 잦으면 Rviz에서 깜빡일 수 있습니다.
        // 적절한 시간을 주어 자연스럽게 갱신되게 합니다.
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        pub_obs_marker_->publish(marker);
    }

    void deleteObstacleMarker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/base_link";
        marker.header.stamp = this->now();
        marker.ns = "mpc_obstacle";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        pub_obs_marker_->publish(marker);
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

        mpc_controller::Input input = solver_.solve(local_current_state, local_ref_traj, detected_obstacles_);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "ego_racecar/base_link";

        // 1. 조향각 설정 (MPC 계산 결과 그대로 사용)
        drive_msg.drive.steering_angle = input.delta; 
        drive_msg.drive.acceleration = input.acc;

        // 2. [핵심] 다이내믹 감속 로직 (Dynamic Speed Scaling)
        // 원리: 조향각이 클수록 타이어의 횡방향 힘이 많이 필요하므로, 종방향 속도를 줄여 슬립을 방지함.

        // 최대 조향각(0.4189 rad) 대비 현재 조향각의 비율 (0.0 ~ 1.0)
        double steering_ratio = std::abs(input.delta) / 0.4189; 
        if (steering_ratio > 1.0) steering_ratio = 1.0; // 안전장치 (1.0 초과 방지)

        // 조향각에 비례하여 감속 (최대 조향 시 설정 속도의 40%로 주행)
        // 0.6이라는 계수는 "감속 강도"입니다. (클수록 코너에서 더 느리게 감)
        double dynamic_speed = target_speed_ * (1.0 - 0.6 * steering_ratio); 

        // 3. 최소 속도 보장 (너무 느려서 멈추는 것 방지)
        // 코너에서도 최소 1.0 m/s 이상은 유지하도록 함
        dynamic_speed = std::max(dynamic_speed, 2.0);

        // 4. 최종 속도 명령 인가
        drive_msg.drive.speed = dynamic_speed;

        pub_drive_->publish(drive_msg);
        auto local_pred_traj = solver_.getPredictedTrajectory();
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