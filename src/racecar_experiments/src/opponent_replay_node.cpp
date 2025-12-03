#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

struct TrajSample {
    double t;      // time [s]
    double x;      // [m]
    double y;      // [m]
    double yaw;    // [rad]
    double speed;  // [m/s]
};

class OpponentReplayNode : public rclcpp::Node {
public:
    OpponentReplayNode()
    : Node("opponent_replay_node")
    {
        // --- Parameters ---
        csv_path_     = declare_parameter<std::string>("csv_path", "spielberg_opponent_scenarioA.csv");
        frame_id_     = declare_parameter<std::string>("frame_id", "map");
        publish_rate_ = declare_parameter<double>("publish_rate", 50.0); // [Hz]

        goal_topic_   = declare_parameter<std::string>("goal_topic", "/goal_pose");

        hb_enable_    = declare_parameter<bool>("heartbeat_enable", true);
        hb_topic_     = declare_parameter<std::string>("heartbeat_topic", "/opp_drive");
        hb_speed_     = declare_parameter<double>("heartbeat_speed", 0.0);
        hb_steer_     = declare_parameter<double>("heartbeat_steer", 0.0);

        if (!load_csv(csv_path_)) {
            RCLCPP_FATAL(get_logger(), "Failed to load trajectory CSV: '%s'", csv_path_.c_str());
            throw std::runtime_error("Failed to load CSV");
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu trajectory samples from '%s'",
                    samples_.size(), csv_path_.c_str());

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

        if (hb_enable_) {
            hb_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(hb_topic_, 10);
            RCLCPP_INFO(get_logger(), "Heartbeat to '%s' ENABLED", hb_topic_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Heartbeat DISABLED");
        }

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&OpponentReplayNode::on_timer, this)
        );
    }

private:
    bool load_csv(const std::string & path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open CSV: %s", path.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string item;
            std::vector<double> vals;
            while (std::getline(ss, item, ',')) {
                try {
                    vals.push_back(std::stod(item));
                } catch (...) {
                    vals.clear();
                    break;
                }
            }

            if (vals.size() < 5) {
                continue; // t, x, y, yaw, speed
            }

            TrajSample s;
            s.t      = vals[0];
            s.x      = vals[1];
            s.y      = vals[2];
            s.yaw    = vals[3];
            s.speed  = vals[4];
            samples_.push_back(s);
        }

        if (samples_.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid samples in CSV");
            return false;
        }
        return true;
    }

    void on_timer() {
        if (samples_.empty()) return;

        if (!start_time_set_) {
            start_time_ = now();
            start_time_set_ = true;
            current_idx_ = 0;
            RCLCPP_INFO(get_logger(), "Opponent replay started.");
        }

        rclcpp::Time now_time = now();
        double elapsed = (now_time - start_time_).seconds();

        TrajSample s_interp;

        if (elapsed >= samples_.back().t) {
            s_interp = samples_.back();
        } else {
            while (current_idx_ + 1 < samples_.size() &&
                   samples_[current_idx_ + 1].t <= elapsed) {
                current_idx_++;
            }

            const auto & s0 = samples_[current_idx_];
            const auto & s1 = samples_[current_idx_ + 1];

            double dt = s1.t - s0.t;
            double alpha = (dt > 0.0) ? (elapsed - s0.t) / dt : 0.0;

            s_interp.t = elapsed;
            s_interp.x = s0.x + alpha * (s1.x - s0.x);
            s_interp.y = s0.y + alpha * (s1.y - s0.y);
            s_interp.speed = s0.speed + alpha * (s1.speed - s0.speed);

            double yaw0 = s0.yaw;
            double yaw1 = s1.yaw;
            double dyaw = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
            s_interp.yaw = yaw0 + alpha * dyaw;
        }

        publish_goal_pose(s_interp, now_time);

        if (hb_enable_) {
            publish_heartbeat(now_time);
        }
    }

    void publish_goal_pose(const TrajSample & s, const rclcpp::Time & stamp) {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = frame_id_;

        goal.pose.position.x = s.x;
        goal.pose.position.y = s.y;
        goal.pose.position.z = 0.0;

        double cy = std::cos(s.yaw * 0.5);
        double sy = std::sin(s.yaw * 0.5);
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = sy;
        goal.pose.orientation.w = cy;

        goal_pub_->publish(goal);
        // 디버그 필요하면:
        // RCLCPP_INFO(get_logger(), "goal (%.2f, %.2f, yaw=%.2f)", s.x, s.y, s.yaw);
    }

    void publish_heartbeat(const rclcpp::Time & stamp) {
        if (!hb_pub_) return;

        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "opp_base";

        msg.drive.speed = hb_speed_;
        msg.drive.steering_angle = hb_steer_;

        hb_pub_->publish(msg);
    }

    // params
    std::string csv_path_;
    std::string frame_id_;
    double publish_rate_;

    std::string goal_topic_;

    bool hb_enable_;
    std::string hb_topic_;
    double hb_speed_;
    double hb_steer_;

    // state
    std::vector<TrajSample> samples_;
    size_t current_idx_{0};
    bool start_time_set_{false};
    rclcpp::Time start_time_;

    // pubs & timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr hb_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OpponentReplayNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Exception in opponent_replay_node: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
