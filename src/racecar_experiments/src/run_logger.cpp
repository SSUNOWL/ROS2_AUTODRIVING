#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp> 
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp> 

#include <fstream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <cstdio> // std::rename 사용을 위해 추가
#include <sstream> // 문자열 조합을 위해 추가

class RunLogger : public rclcpp::Node
{
public:
    RunLogger() : Node("run_logger"), timer_state_(IDLE), has_path_(false), has_left_start_(false)
    {
        // ===== 파라미터 설정 =====
        output_dir_ = this->declare_parameter<std::string>("output_dir", "logs");
        scenario_name_ = this->declare_parameter<std::string>("scenario_name", "unknown_scenario");
        planner_mode_ = this->declare_parameter<std::string>("planner_mode", "FRENET");
        collision_topic_ = this->declare_parameter<std::string>("collision_topic", "/collision");
        this->declare_parameter("goal_tolerance", 0.5); 

        // 파일 이름 생성 (Timestamp 포함)
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now_c));
        std::string ts(buf);

        // [수정] 나중에 이름을 바꾸기 위해 멤버 변수에 저장
        log_filename_ = output_dir_ + "/" + scenario_name_ + "_" +
                               planner_mode_ + "_" + ts + ".csv";

        ofs_.open(log_filename_);
        if (!ofs_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s", log_filename_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging to %s (Waiting for start...)", log_filename_.c_str());
            ofs_ << "t,x,y,yaw,speed,yaw_rate,a_lat,collision,timer_state,elapsed_time\n";
        }

        // 초기화
        start_time_ = -1.0;
        last_time_ = -1.0;
        max_speed_ = 0.0;
        max_a_lat_ = 0.0;
        collision_count_ = 0;
        collision_flag_ = 0;
        total_dist_ = 0.0;
        last_x_ = last_y_ = NAN;
        should_shutdown_ = false; 
        shutdown_reason_ = "N/A"; 

        // 구독 및 발행
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&RunLogger::odomCallback, this, std::placeholders::_1));

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RunLogger::cmdCallback, this, std::placeholders::_1));

        collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            collision_topic_, 10,
            std::bind(&RunLogger::collisionCallback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&RunLogger::pathCallback, this, std::placeholders::_1));
        
        pub_time_ = this->create_publisher<std_msgs::msg::Float32>("/experiments/travel_time", 10);
        pub_goal_ = this->create_publisher<std_msgs::msg::Bool>("/experiments/goal_reached", 10);

        shutdown_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RunLogger::checkShutdown, this));
    }

    ~RunLogger()
    {
        summarize();
        // summarize에서 파일을 닫으므로 여기서는 중복 닫기 방지 체크
        if (ofs_.is_open()) {
            ofs_.close();
        }
    }

private:
    enum State { IDLE, RUNNING, FINISHED };

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = std::hypot(msg->linear.x, msg->linear.y);
        current_speed_cmd_ = v;
        current_yaw_rate_cmd_ = msg->angular.z;
    }
    
    bool prev_collision_state_ = false;

    void collisionCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool now = msg->data;
        if (!prev_collision_state_ && now) {
            collision_count_++;
            if (!should_shutdown_) { 
                RCLCPP_WARN(this->get_logger(), "Collision detected! Shutting down.");
                should_shutdown_ = true;
                shutdown_reason_ = "Collision Detected";
            }
        }
        collision_flag_ = now ? 1 : 0;
        prev_collision_state_ = now;
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (should_shutdown_) return; 
        
        if (msg->poses.empty() || msg->poses.size() < 10) return; 

        double sx = msg->poses.front().pose.position.x;
        double sy = msg->poses.front().pose.position.y;
        double gx = msg->poses.back().pose.position.x;
        double gy = msg->poses.back().pose.position.y;
        
        if (timer_state_ == IDLE || timer_state_ == FINISHED)
        {
            start_x_ = sx;
            start_y_ = sy;
            timer_state_ = IDLE;
            has_left_start_ = false;

            auto goal_msg = std_msgs::msg::Bool();
            goal_msg.data = false;
            pub_goal_->publish(goal_msg);
            
            RCLCPP_INFO(this->get_logger(), "New Path Set! Ready to Start. (%.2f, %.2f) -> (%.2f, %.2f)", 
                start_x_, start_y_, gx, gy);
        }

        goal_x_ = gx;
        goal_y_ = gy;
        has_path_ = true;
    }

    void checkShutdown()
    {
        if (should_shutdown_) {
            RCLCPP_WARN(this->get_logger(), "Initiating node shutdown: %s", shutdown_reason_.c_str());
            shutdown_check_timer_->cancel(); 
            rclcpp::shutdown(); 
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (should_shutdown_) return;

        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        if (start_time_ < 0.0) {
            start_time_ = t;
            last_time_ = t;
        }
        
        last_time_ = t;
        double rel_t = t - start_time_;

        const auto &q = msg->pose.pose.orientation;
        double yaw = quaternionToYaw(q.x, q.y, q.z, q.w);
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double speed_odom = std::hypot(vx, vy);
        double yaw_rate_odom = msg->twist.twist.angular.z;

        double speed = (speed_odom > 0.01) ? speed_odom : current_speed_cmd_;
        double yaw_rate = (std::fabs(yaw_rate_odom) > 1e-3) ? yaw_rate_odom : current_yaw_rate_cmd_;
        double a_lat = speed * yaw_rate;

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (timer_state_ == IDLE || timer_state_ == RUNNING) {
            this->checkGoalReach(x, y, speed);
        }

        double elapsed = 0.0;
        if (timer_state_ == RUNNING) {
            elapsed = (this->get_clock()->now() - timer_start_time_).seconds();
            
            if (speed > max_speed_) max_speed_ = speed;
            if (std::fabs(a_lat) > max_a_lat_) max_a_lat_ = std::fabs(a_lat);
            if (!std::isnan(last_x_)) {
                total_dist_ += std::hypot(x - last_x_, y - last_y_);
            }

        } else if (timer_state_ == FINISHED) {
             elapsed = (timer_goal_time_ - timer_start_time_).seconds();
        }
        
        last_x_ = x;
        last_y_ = y;

        if (ofs_.is_open() && timer_state_ != IDLE) {
            ofs_ << std::fixed << std::setprecision(4)
                 << rel_t << ","
                 << x << "," << y << ","
                 << yaw << ","
                 << speed << ","
                 << yaw_rate << ","
                 << a_lat << ","
                 << collision_flag_ << ","
                 << timer_state_ << "," 
                 << elapsed << "\n";
        }
    }
    
    void checkGoalReach(double x, double y, double speed)
    {
        if (!has_path_) return;

        double tolerance = this->get_parameter("goal_tolerance").as_double();
        double dist_to_start = std::sqrt(std::pow(x - start_x_, 2) + std::pow(y - start_y_, 2));
        double dist_to_goal = std::sqrt(std::pow(x - goal_x_, 2) + std::pow(y - goal_y_, 2));

        switch (timer_state_)
        {
            case IDLE:
                if (dist_to_start < 2.0 && std::abs(speed) > 0.1)
                {
                    timer_start_time_ = this->get_clock()->now();
                    timer_state_ = RUNNING;
                    has_left_start_ = false; 
                    RCLCPP_INFO(this->get_logger(), "Timer Started! Logging begins.");
                }
                break;

            case RUNNING:
            {
                if (!has_left_start_) {
                    if (dist_to_start > 3.0) {
                        has_left_start_ = true;
                    }
                }

                if (has_left_start_ && dist_to_goal < tolerance)
                {
                    double elapsed = (this->get_clock()->now() - timer_start_time_).seconds();
                    timer_state_ = FINISHED; 
                    timer_goal_time_ = this->get_clock()->now(); 

                    RCLCPP_INFO(this->get_logger(), "GOAL REACHED! Time: %.4f s", elapsed);
                    
                    auto time_msg = std_msgs::msg::Float32();
                    time_msg.data = elapsed;
                    pub_time_->publish(time_msg);

                    auto goal_msg = std_msgs::msg::Bool();
                    goal_msg.data = true;
                    pub_goal_->publish(goal_msg);
                    
                    if (!should_shutdown_) {
                        should_shutdown_ = true;
                        shutdown_reason_ = "Goal Reached";
                    }
                }
                break;
            }
            default: break;
        }
    }


    double quaternionToYaw(double x, double y, double z, double w)
    {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void summarize()
    {
        double total_time = last_time_ - start_time_;
        bool invalid_run = false;
        if (total_dist_ < 1.0 && collision_count_ == 0 && timer_state_ != FINISHED) {
            invalid_run = true;
        }

        RCLCPP_INFO(this->get_logger(), "===== RUN SUMMARY (%s) =====", shutdown_reason_.c_str());
        RCLCPP_INFO(this->get_logger(), "Result State: %s", (timer_state_ == FINISHED) ? "SUCCESS" : "FAIL/INCOMPLETE");
        RCLCPP_INFO(this->get_logger(), "Total Logged Time: %.3f s", total_time);
        
        if (timer_state_ == FINISHED) {
             double run_time = (timer_goal_time_ - timer_start_time_).seconds();
             RCLCPP_INFO(this->get_logger(), "Lap Time: %.3f s", run_time);
        }
        RCLCPP_INFO(this->get_logger(), "Travel Dist: %.3f m", total_dist_);
        RCLCPP_INFO(this->get_logger(), "Max Speed: %.3f m/s", max_speed_);
        RCLCPP_INFO(this->get_logger(), "Max Lat Acc: %.3f m/s^2", max_a_lat_);
        RCLCPP_INFO(this->get_logger(), "Collision: %d", collision_count_);
        RCLCPP_INFO(this->get_logger(), "Invalid Run (Stuck?): %s", invalid_run ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "======================================");

        // [추가됨] 파일 이름 변경 로직
        // 파일을 안전하게 닫고 나서 이름을 변경해야 합니다.
        if (ofs_.is_open()) {
            ofs_.close();
        }

        // 완주에 성공했을 경우에만 파일 이름에 시간 추가
        if (timer_state_ == FINISHED) {
            double run_time = (timer_goal_time_ - timer_start_time_).seconds();
            
            // 새 파일 이름 만들기: .csv 확장자를 찾아서 그 앞에 시간을 삽입
            std::string new_filename = log_filename_;
            size_t ext_pos = new_filename.find_last_of(".");
            
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << "_time_" << run_time << "s";
            
            if (ext_pos != std::string::npos) {
                new_filename.insert(ext_pos, ss.str());
            } else {
                new_filename += ss.str(); // 확장자가 없으면 그냥 뒤에 붙임
            }

            // 파일 이름 변경 (rename)
            if (std::rename(log_filename_.c_str(), new_filename.c_str()) == 0) {
                RCLCPP_INFO(this->get_logger(), "Log file renamed to include lap time: %s", new_filename.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to rename log file.");
            }
        }
    }

    std::string output_dir_;
    std::string scenario_name_;
    std::string planner_mode_;
    std::string collision_topic_;
    std::string log_filename_; // [추가] 초기 파일명 저장용

    std::ofstream ofs_;
    double start_time_, last_time_;
    double max_speed_, max_a_lat_;
    int collision_count_, collision_flag_;
    double total_dist_, last_x_, last_y_;
    double current_speed_cmd_ = 0.0;
    double current_yaw_rate_cmd_ = 0.0;
    
    bool should_shutdown_;
    std::string shutdown_reason_;
    rclcpp::TimerBase::SharedPtr shutdown_check_timer_;
    
    State timer_state_;
    bool has_path_;
    bool has_left_start_; 
    double start_x_, start_y_, goal_x_, goal_y_;
    rclcpp::Time timer_start_time_, timer_goal_time_; 

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_time_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RunLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}