#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

struct TeleopSample {
    double t;      // time [s]
    double speed;  // [m/s]
    double steer;  // [rad]
};

class OpponentReplayNode : public rclcpp::Node {
public:
    OpponentReplayNode()
    : Node("opponent_replay_node")
    {
        // --- Parameters ---
        csv_path_     = declare_parameter<std::string>("csv_path", "opp_teleop_traj.csv");
        publish_rate_ = declare_parameter<double>("publish_rate", 50.0); // [Hz]

        hb_enable_    = declare_parameter<bool>("enable_replay", true);
        hb_topic_     = declare_parameter<std::string>("drive_topic", "/opp_drive");

        // 재생 시 튜닝용
        speed_scale_  = declare_parameter<double>("speed_scale", 1.0);  // 전체 속도 배율
        max_steer_    = declare_parameter<double>("max_steer", 0.4);    // 최대 조향각
        steer_smooth_alpha_ = declare_parameter<double>("steer_smooth_alpha", 0.5); // 조향 필터

        if (!load_csv(csv_path_)) {
            RCLCPP_FATAL(get_logger(), "Failed to load teleop CSV: '%s'", csv_path_.c_str());
            throw std::runtime_error("Failed to load CSV");
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu teleop samples from '%s'",
                    samples_.size(), csv_path_.c_str());

        if (hb_enable_) {
            drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(hb_topic_, 10);
            RCLCPP_INFO(get_logger(), "Drive replay to '%s' ENABLED", hb_topic_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Drive replay DISABLED");
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
        size_t line_no = 0;
        while (std::getline(file, line)) {
            ++line_no;
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

            if (vals.empty()) {
                continue;
            }

            TeleopSample s{};

            if (vals.size() == 3) {
                // time_s, speed_mps, steer_rad
                s.t     = vals[0];
                s.speed = vals[1];
                s.steer = vals[2];
            } else if (vals.size() >= 6) {
                // time, x, y, yaw, speed, steer 형태도 호환
                s.t     = vals[0];
                s.speed = vals[4];
                s.steer = vals[5];
            } else {
                RCLCPP_WARN(get_logger(),
                            "Line %zu: unexpected column size %zu, skipping",
                            line_no, vals.size());
                continue;
            }

            samples_.push_back(s);
        }

        if (samples_.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid samples in CSV");
            return false;
        }

        // 시간 기준으로 정렬 (혹시 순서 섞여 있을 경우 대비)
        std::sort(samples_.begin(), samples_.end(),
                  [](const TeleopSample &a, const TeleopSample &b){
                      return a.t < b.t;
                  });

        return true;
    }

    void on_timer() {
        if (!hb_enable_ || samples_.empty()) return;

        if (!start_time_set_) {
            start_time_ = now();
            start_time_set_ = true;
            current_idx_ = 0;
            prev_steer_filtered_ = samples_.front().steer;
            RCLCPP_INFO(get_logger(), "Teleop replay started.");
        }

        rclcpp::Time now_time = now();
        double elapsed = (now_time - start_time_).seconds();

        TeleopSample s_interp;

        // 재생 종료 이후에는 마지막 명령 유지 (혹은 speed=0으로 바꾸고 싶으면 여기서 처리)
        if (elapsed >= samples_.back().t) {
            s_interp = samples_.back();
        } else {
            // elapsed에 맞는 구간 찾기
            while (current_idx_ + 1 < samples_.size() &&
                   samples_[current_idx_ + 1].t <= elapsed) {
                current_idx_++;
            }

            const auto & s0 = samples_[current_idx_];
            const auto & s1 = samples_[current_idx_ + 1];

            double dt = s1.t - s0.t;
            double alpha = (dt > 0.0) ? (elapsed - s0.t) / dt : 0.0;

            s_interp.t     = elapsed;
            s_interp.speed = s0.speed + alpha * (s1.speed - s0.speed);
            s_interp.steer = s0.steer + alpha * (s1.steer - s0.steer);
        }

        publish_drive(s_interp, now_time);
    }

    void publish_drive(const TeleopSample & s, const rclcpp::Time & stamp) {
        if (!drive_pub_) return;

        double speed_cmd  = s.speed * speed_scale_;
        double steer_cmd  = s.steer;

        // 조향각 clamp
        if (steer_cmd > max_steer_)  steer_cmd = max_steer_;
        if (steer_cmd < -max_steer_) steer_cmd = -max_steer_;

        // 간단한 1차 필터로 조향 스무딩
        double alpha = steer_smooth_alpha_;
        double steer_filtered = prev_steer_filtered_ + alpha * (steer_cmd - prev_steer_filtered_);
        prev_steer_filtered_ = steer_filtered;

        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "opp_base";

        msg.drive.speed = speed_cmd;
        msg.drive.steering_angle = steer_filtered;

        drive_pub_->publish(msg);
    }

    // params
    std::string csv_path_;
    double publish_rate_;

    bool   hb_enable_;
    std::string hb_topic_;

    double speed_scale_;
    double max_steer_;
    double steer_smooth_alpha_;

    // state
    std::vector<TeleopSample> samples_;
    size_t current_idx_{0};
    bool start_time_set_{false};
    rclcpp::Time start_time_;

    double prev_steer_filtered_{0.0};

    // pub & timer
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
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
