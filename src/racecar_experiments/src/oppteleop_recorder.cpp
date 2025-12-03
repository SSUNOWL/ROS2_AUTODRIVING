#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <string>

class OppTeleopRecorder : public rclcpp::Node {
public:
    OppTeleopRecorder()
    : Node("opp_teleop_recorder")
    {
        // --- Parameters ---
        teleop_topic_ = declare_parameter<std::string>("teleop_topic", "/opp_teleop");
        csv_path_     = declare_parameter<std::string>("csv_path", "opp_teleop_traj.csv");
        min_dt_       = declare_parameter<double>("min_dt", 0.02); // [s]

        file_.open(csv_path_, std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_FATAL(get_logger(), "Failed to open CSV file: '%s'", csv_path_.c_str());
            throw std::runtime_error("Cannot open CSV");
        }

        // time, speed, steer
        file_ << "# time_s,speed_mps,steer_rad\n";

        sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            teleop_topic_, 10,
            std::bind(&OppTeleopRecorder::driveCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(get_logger(), "Recording opp teleop from '%s' to '%s'",
                    teleop_topic_.c_str(), csv_path_.c_str());
    }

    ~OppTeleopRecorder() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        rclcpp::Time stamp = msg->header.stamp;

        if (!start_time_set_) {
            start_time_ = stamp;
            last_write_time_ = stamp;
            start_time_set_ = true;
            RCLCPP_INFO(get_logger(), "Start time set: %.3f", start_time_.seconds());
        }

        double t  = (stamp - start_time_).seconds();
        double dt = (stamp - last_write_time_).seconds();

        if (dt < min_dt_) {
            return;
        }
        last_write_time_ = stamp;

        double speed  = msg->drive.speed;
        double steer  = msg->drive.steering_angle;

        file_ << t << "," << speed << "," << steer << "\n";
        // file_.flush(); // 필요하면 켜기
    }

    std::string teleop_topic_;
    std::string csv_path_;
    double min_dt_;

    std::ofstream file_;

    bool start_time_set_{false};
    rclcpp::Time start_time_;
    rclcpp::Time last_write_time_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OppTeleopRecorder>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Exception in opp_teleop_recorder: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
