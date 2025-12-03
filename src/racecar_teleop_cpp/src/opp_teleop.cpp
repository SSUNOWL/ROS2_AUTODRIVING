#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <thread>
#include <mutex>
#include <cmath>
#include <iomanip>
#include <sys/select.h>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71
#define KEYCODE_K 0x6b // [추가] K 키 코드 (ASCII)

const char* msg = R"(
---------------------------
Moving around:
        W
   A    S    D
        X

W: Increase Speed
S: Stop (Zero Speed)
A: Steer Left
D: Steer Right
X: Decrease Speed (Reverse)
K: Center Steering (Reset) 

CTRL-C to quit
---------------------------
)";

class OppenetTeleop : public rclcpp::Node
{
public:
    OppenetTeleop() : Node("oppenet_teleop")
    {
        // --- Parameters for topics ---
        // 실제 차량 제어용 토픽 (기존 /opp_drive 유지)
        std::string drive_topic  = this->declare_parameter<std::string>("drive_topic",  "/opp_drive");
        // 텔레옵 명령 기록/리플레이용 토픽
        std::string teleop_topic = this->declare_parameter<std::string>("teleop_topic", "/opp_teleop");

        // 1. 명령 퍼블리셔
        pub_drive_  = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        pub_teleop_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(teleop_topic, 10);

        // 2. 오돔 서브스크라이버
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/opp_racecar/odom", 10, 
            std::bind(&OppenetTeleop::odom_callback, this, std::placeholders::_1));

        // 3. 충돌 감지 서브스크라이버
        sub_crash_ = this->create_subscription<std_msgs::msg::Bool>(
            "/experiments/crash_detected", 10,
            std::bind(&OppenetTeleop::crash_callback, this, std::placeholders::_1));

        target_speed_ = 0.0;
        target_steering_angle_ = 0.0;
        
        actual_speed_ = 0.0;
        estimated_steering_angle_ = 0.0;
        
        is_emergency_ = false;
        last_crash_msg_time_ = this->now(); // 시간 초기화

        RCLCPP_INFO(this->get_logger(), "Oppenet Teleop Node Started (Auto-Resume Enabled)");
        RCLCPP_INFO(this->get_logger(), "  drive_topic  : %s", drive_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  teleop_topic : %s", teleop_topic.c_str());
    }

    // 충돌 감지 콜백
    void crash_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 메시지 수신 시간 갱신 (Heartbeat 역할)
        last_crash_msg_time_ = this->now();

        if (msg->data) {
            // 충돌 발생! (True)
            if (!is_emergency_) {
                is_emergency_ = true;
                target_speed_ = 0.0; // 즉시 정지
                RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP! (Crash Detected)");
            }
        } else {
            // 충돌 해제 (False)
            if (is_emergency_) {
                is_emergency_ = false;
                RCLCPP_INFO(this->get_logger(), "Crash Cleared. Resuming control...");
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        actual_speed_ = msg->twist.twist.linear.x;
        double angular_z = msg->twist.twist.angular.z; 

        double wheelbase = 0.33; 
        double raw_estimate = 0.0;

        if (std::abs(actual_speed_) > 0.5) {
            raw_estimate = std::atan((wheelbase * angular_z) / actual_speed_);
        } else {
            raw_estimate = 0.0;
        }

        raw_estimate = std::clamp(raw_estimate, -0.4, 0.4);
        double alpha = 0.1; 
        estimated_steering_angle_ = (alpha * raw_estimate) + ((1.0 - alpha) * estimated_steering_angle_);
    }

    void keyLoop()
    {
        char c;
        
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts(msg);

        while (rclcpp::ok())
        {
            fd_set set;
            FD_ZERO(&set);
            FD_SET(kfd, &set);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 0.1s

            int res = select(kfd + 1, &set, NULL, NULL, &timeout);
            bool steering_input_active = false;

            // 타임아웃 체크 (충돌 노드 죽었거나 메시지 안 오면 자동 해제)
            rclcpp::Time now = this->now();
            double time_diff = (now - last_crash_msg_time_).seconds();
            
            // 5초 이상 충돌 메시지가 없으면 안전한 것으로 간주
            if (is_emergency_ && time_diff > 5.0) {
                is_emergency_ = false;
                RCLCPP_INFO(this->get_logger(), "Collision Monitor Timeout. Resuming control...");
            }
            
            if (res > 0)
            {
                if (read(kfd, &c, 1) < 0) {
                    perror("read():");
                    exit(-1);
                }

                if (!is_emergency_) {
                    // 속도에 따른 조향 변화량(step) 계산
                    double current_speed_abs = std::abs(target_speed_);
                    double steer_step = 0.2 / (1.0 + (current_speed_abs * 0.5)); 
                    steer_step = std::max(0.02, steer_step);

                    switch(c)
                    {
                        case KEYCODE_W: target_speed_ += 0.5; break;
                        case KEYCODE_S: target_speed_ = 0.0; target_steering_angle_ = 0.0; break;
                        case KEYCODE_X: target_speed_ -= 0.5; break;
                        
                        case KEYCODE_A: 
                            target_steering_angle_ += steer_step; 
                            steering_input_active = true; 
                            break;
                        case KEYCODE_D: 
                            target_steering_angle_ -= steer_step; 
                            steering_input_active = true; 
                            break;
                        case KEYCODE_K: 
                            target_steering_angle_ = 0.0; 
                            steering_input_active = true; 
                            break;
                            
                        case KEYCODE_Q: 
                            RCLCPP_INFO(this->get_logger(), "Exiting..."); 
                            return;
                    }
                } else {
                    // 비상 상태일 땐 키 입력은 받되 속도는 0으로 유지
                    switch(c)
                    {
                        case KEYCODE_S: 
                            target_speed_ = 0.0; 
                            target_steering_angle_ = 0.0; 
                            break;
                        case KEYCODE_Q: 
                            RCLCPP_INFO(this->get_logger(), "Exiting..."); 
                            return;
                    }
                }
            }

            // Auto-Centering (직진 복귀 기능)
            if (!steering_input_active && std::abs(target_steering_angle_) > 0.001)
            {
                double center_speed = 0.015 + (std::abs(target_speed_) * 0.005); 
                
                if (target_steering_angle_ > 0) 
                    target_steering_angle_ = std::max(0.0, target_steering_angle_ - center_speed);
                else 
                    target_steering_angle_ = std::min(0.0, target_steering_angle_ + center_speed);
            }

            // Saturation for speed
            if (target_speed_ > 7.0) target_speed_ = 7.0;
            if (target_speed_ < -5.0) target_speed_ = -5.0;

            // 속도에 따른 최대 조향각 제한 (Dynamic Steering Limit)
            double max_steering_at_current_speed = 0.4 / (1.0 + (std::abs(target_speed_) * 0.2));
            max_steering_at_current_speed = std::max(0.1, max_steering_at_current_speed);

            if (target_steering_angle_ > max_steering_at_current_speed) 
                target_steering_angle_ = max_steering_at_current_speed;
            if (target_steering_angle_ < -max_steering_at_current_speed) 
                target_steering_angle_ = -max_steering_at_current_speed;

            // 비상 정지 강제 적용
            double final_speed = target_speed_;
            if (is_emergency_) {
                final_speed = 0.0;
            }

            // 명령 메시지 생성
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "base_link";
            drive_msg.drive.speed = final_speed;
            drive_msg.drive.steering_angle = target_steering_angle_;

            // 🔥 실제 제어용 /opp_drive 토픽으로 퍼블리시
            pub_drive_->publish(drive_msg);

            // 🔥 동일한 내용을 텔레옵 기록용 토픽(/opp_teleop)으로도 퍼블리시
            pub_teleop_->publish(drive_msg);

            // 상태 출력
            print_status();
        }
    }

    ~OppenetTeleop()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    void print_status()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        std::string status_str = "[NORMAL]";
        if (is_emergency_) status_str = "\033[1;31m[!! EMERGENCY !!]\033[0m"; // 빨간색 텍스트

        std::cout << "\033[2K\r" 
                  << status_str 
                  << std::fixed << std::setprecision(2)
                  << " CMD Spd:" << (is_emergency_ ? 0.0 : target_speed_) 
                  << " Str:" << target_steering_angle_
                  << " | REAL Spd:" << actual_speed_ 
                  << std::flush;
    }

    // 🔥 퍼블리셔 2개: 실제 제어용 / 기록용
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_teleop_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_crash_;
    
    double target_speed_;
    double target_steering_angle_;
    double actual_speed_;
    double estimated_steering_angle_;
    
    bool is_emergency_;
    rclcpp::Time last_crash_msg_time_; // 타임아웃 계산용

    std::mutex data_mutex_;
    int kfd = 0;
    struct termios cooked, raw;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OppenetTeleop>();

    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    node->keyLoop();

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    
    return 0;
}
