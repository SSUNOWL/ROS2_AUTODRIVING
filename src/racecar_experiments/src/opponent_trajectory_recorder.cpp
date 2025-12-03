#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fstream>
#include <string>
#include <cmath>

class OpponentTrajectoryRecorder : public rclcpp::Node {
public:
    OpponentTrajectoryRecorder()
    : Node("opponent_trajectory_recorder")
    {
        // --- Parameters ---
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/opponent/odom");
        csv_path_   = declare_parameter<std::string>("csv_path", "opponent_traj.csv");
        min_dt_     = declare_parameter<double>("min_dt", 0.02); // [s], 0.02s → 최대 50Hz 수준

        file_.open(csv_path_, std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_FATAL(get_logger(), "Failed to open CSV file for writing: '%s'", csv_path_.c_str());
            throw std::runtime_error("Cannot open CSV");
        }

        // 헤더 한 줄 써두기 (필요 없으면 지워도 됨)
        file_ << "# time_s,x_m,y_m,yaw_rad,speed_mps\n";

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&OpponentTrajectoryRecorder::odomCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Recording opponent trajectory from '%s' to '%s'",
                    odom_topic_.c_str(), csv_path_.c_str());
    }

    ~OpponentTrajectoryRecorder() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;

        // 최초 시간 기준 설정
        if (!start_time_set_) {
            start_time_ = stamp;
            last_write_time_ = stamp;
            start_time_set_ = true;
            RCLCPP_INFO(get_logger(), "Start time set: %.3f", start_time_.seconds());
        }

        double t = (stamp - start_time_).seconds();
        double dt = (stamp - last_write_time_).seconds();

        // 너무 촘촘하면 min_dt 기준으로 다운샘플링
        if (dt < min_dt_) {
            return;
        }
        last_write_time_ = stamp;

        // 위치
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // orientation → yaw
        const auto & q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // 속도 (world 기준)
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double speed = std::sqrt(vx * vx + vy * vy);

        // CSV에 쓰기
        file_ << t << "," << x << "," << y << "," << yaw << "," << speed << "\n";

        // (선택) flush 자주 하고 싶으면 이거 켜기
        // file_.flush();
    }

    // Params
    std::string odom_topic_;
    std::string csv_path_;
    double min_dt_;

    // File
    std::ofstream file_;

    // Time
    bool start_time_set_{false};
    rclcpp::Time start_time_;
    rclcpp::Time last_write_time_;

    // ROS
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OpponentTrajectoryRecorder>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Exception in opponent_trajectory_recorder: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
