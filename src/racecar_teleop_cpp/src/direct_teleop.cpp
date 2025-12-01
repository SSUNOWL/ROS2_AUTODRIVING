#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <thread>
#include <mutex>
#include <cmath>
#include <iomanip>
#include <sys/select.h>
#include <algorithm> // std::max, std::min, std::clamp 사용

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// 키 코드 정의
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78
#define KEYCODE_Q 0x71

// 안내 메시지
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

CTRL-C to quit
---------------------------
)";

class DirectTeleop : public rclcpp::Node
{
public:
    DirectTeleop() : Node("direct_teleop")
    {
        // 1. 명령 퍼블리셔
        pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        
        // 2. 실제 속도/각도 추정을 위한 서브스크라이버
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecR/odom", 10, 
            std::bind(&DirectTeleop::odom_callback, this, std::placeholders::_1));

        target_speed_ = 0.0;
        target_steering_angle_ = 0.0;
        
        actual_speed_ = 0.0;
        estimated_steering_angle_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "direct Teleop Node Started");
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 1. 실제 선속도 업데이트
        actual_speed_ = msg->twist.twist.linear.x;
        double angular_z = msg->twist.twist.angular.z; // Yaw Rate (rad/s)

        // 2. [Feedback] 실제 차량의 움직임을 기반으로 조향각 역산 (Estimation)
        // 공식: tan(steer) = (wheelbase * yaw_rate) / velocity
        double wheelbase = 0.33; 
        double raw_estimate = 0.0;

        // [유지] 저속 주행 시 노이즈 방지 (화면 표시용)
        if (std::abs(actual_speed_) > 0.5) {
            raw_estimate = std::atan((wheelbase * angular_z) / actual_speed_);
        } else {
            raw_estimate = 0.0;
        }

        // [유지] 화면 표시용 클램핑 및 필터
        raw_estimate = std::clamp(raw_estimate, -0.4, 0.4);
        double alpha = 0.1; 
        estimated_steering_angle_ = (alpha * raw_estimate) + ((1.0 - alpha) * estimated_steering_angle_);
    }

    void keyLoop()
    {
        char c;
        
        // 터미널 설정 (Raw 모드)
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
            timeout.tv_usec = 100000; // 0.1초 (루프 주기)

            int res = select(kfd + 1, &set, NULL, NULL, &timeout);

            bool steering_input_active = false; // 이번 루프에서 조향 키를 눌렀는지 확인

            if (res > 0)
            {
                if (read(kfd, &c, 1) < 0) {
                    perror("read():");
                    exit(-1);
                }

                switch(c)
                {
                    case KEYCODE_W:
                        target_speed_ += 0.5;
                        break;
                    case KEYCODE_S:
                        target_speed_ = 0.0;
                        target_steering_angle_ = 0.0;
                        break;
                    case KEYCODE_X:
                        target_speed_ -= 0.5;
                        break;
                    case KEYCODE_A:
                        // 조향 민감도 (빠른 조향)
                        target_steering_angle_ += 0.2; 
                        steering_input_active = true;
                        break;
                    case KEYCODE_D:
                        // 조향 민감도
                        target_steering_angle_ -= 0.2;
                        steering_input_active = true;
                        break;
                    case KEYCODE_Q:
                        RCLCPP_INFO(this->get_logger(), "Exiting...");
                        return;
                }
            }

            // ---------------------------------------------------------
            // [기능 추가] 핸들 자동 풀림 (Auto-Centering)
            // ---------------------------------------------------------
            if (!steering_input_active && std::abs(target_steering_angle_) > 0.001)
            {
                // 복원 속도 (천천히 풀림)
                double center_speed = 0.015; 

                if (target_steering_angle_ > 0)
                    target_steering_angle_ = std::max(0.0, target_steering_angle_ - center_speed);
                else
                    target_steering_angle_ = std::min(0.0, target_steering_angle_ + center_speed);
            }

            // 제한 (Saturation) - 단순 최대/최소 제한으로 복귀
            if (target_speed_ > 7.0) target_speed_ = 7.0;
            if (target_speed_ < -5.0) target_speed_ = -5.0;
            if (target_steering_angle_ > 0.4) target_steering_angle_ = 0.4;
            if (target_steering_angle_ < -0.4) target_steering_angle_ = -0.4;

            // 명령 발행
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "base_link";
            drive_msg.drive.speed = target_speed_;
            drive_msg.drive.steering_angle = target_steering_angle_;
            pub_drive_->publish(drive_msg);

            // 상태 출력
            print_status();
        }
    }

    ~DirectTeleop()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    void print_status()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 화면 갱신
        std::cout << "\033[2K\r" 
                  << std::fixed << std::setprecision(2)
                  << "[CMD] Spd:" << target_speed_ 
                  << " Str:" << target_steering_angle_
                  << " > "
                  << "[REAL] Spd:" << actual_speed_ 
                  << " Est.Str:" << estimated_steering_angle_
                  << std::flush;
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    
    double target_speed_;
    double target_steering_angle_;
    
    double actual_speed_;
    double estimated_steering_angle_;

    std::mutex data_mutex_;

    int kfd = 0;
    struct termios cooked, raw;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectTeleop>();

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