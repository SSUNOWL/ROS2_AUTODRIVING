#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

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
F1TENTH C++ Direct Controller
---------------------------
Moving around:
        W
   A    S    D

W: Increase Speed
S: Stop (Zero Speed)
A: Steer Left
D: Steer Right
X: Decrease Speed (Reverse)

CTRL-C to quit
---------------------------
)";

class OppenetTeleop : public rclcpp::Node
{
public:
    OppenetTeleop() : Node("oppenet_teleop")
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/opp_drive", 10);
        speed_ = 0.0;
        steering_angle_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Oppenet Teleop Node Started");
    }

    // 키보드 입력을 받는 메인 루프
    void keyLoop()
    {
        char c;
        bool dirty = false;

        // 터미널 설정 가져오기 (Raw 모드 전환을 위해)
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts(msg);

        while (rclcpp::ok())
        {
            // 키 입력 읽기 (Blocking 방식이지만 read로 1바이트씩 읽음)
            if (read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            // 키 입력 처리
            switch(c)
            {
                case KEYCODE_W:
                    speed_ += 0.5;
                    dirty = true;
                    break;
                case KEYCODE_S:
                    speed_ = 0.0;
                    steering_angle_ = 0.0;
                    dirty = true;
                    break;
                case KEYCODE_X:
                    speed_ -= 0.5;
                    dirty = true;
                    break;
                case KEYCODE_A:
                    steering_angle_ += 0.1;
                    dirty = true;
                    break;
                case KEYCODE_D:
                    steering_angle_ -= 0.1;
                    dirty = true;
                    break;
                case KEYCODE_Q: // q를 눌러도 종료
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    return;
            }

            // 속도 및 조향각 제한
            if (speed_ > 5.0) speed_ = 5.0;
            if (speed_ < -3.0) speed_ = -3.0;
            if (steering_angle_ > 0.4) steering_angle_ = 0.4;
            if (steering_angle_ < -0.4) steering_angle_ = -0.4;

            // 메시지 발행
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = speed_;
            drive_msg.drive.steering_angle = steering_angle_;
            publisher_->publish(drive_msg);

            if (dirty)
            {
                std::cout << "\rSpeed: " << speed_ << "\tSteer: " << steering_angle_ << "     " << std::flush;
                dirty = false;
            }
        }
    }

    // 소멸자: 프로그램 종료 시 터미널 설정을 원상복구해야 함
    ~OppenetTeleop()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    double speed_;
    double steering_angle_;
    int kfd = 0;
    struct termios cooked, raw;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OppenetTeleop>();
    
    // 루프 실행
    node->keyLoop();

    rclcpp::shutdown();
    return 0;
}