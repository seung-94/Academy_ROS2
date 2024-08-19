#include "rclcpp/rclcpp.hpp"           // ROS2 C++ 클라이언트 라이브러리의 헤더 파일을 포함합니다.
#include "sensor_msgs/msg/image.hpp"   // 이미지 메시지를 처리하기 위한 헤더 파일을 포함합니다.
#include "geometry_msgs/msg/twist.hpp" // 로봇의 속도 명령(Twist 메시지)을 다루기 위한 헤더 파일을 포함합니다.
#include "zbar.h"                      // QR 코드를 인식하기 위한 zbar 라이브러리 헤더 파일 포함

// LineTracingRobot 클래스는 rclcpp::Node를 상속받아 ROS2 노드를 생성합니다.
// ROS2 노드는 네트워크를 통해 데이터를 주고받기 위한 독립 실행형 프로그램입니다.
class LineTracingRobot : public rclcpp::Node
{
public:
    // 생성자: 클래스가 인스턴스화될 때 호출됩니다.
    // 여기서 노드의 이름을 지정하고, 카메라 데이터를 구독하고, 로봇의 속도를 제어하기 위한 설정을 합니다.
    LineTracingRobot() : Node("line_tracing_robot"), initial_move_done_(false), waiting_after_move_(false), qr_code_detected_(false), start_point_(0)
    {
        // 카메라에서 이미지를 수신하기 위해 "/camera/image_raw" 토픽을 구독합니다.
        // 이 토픽에는 카메라로부터 전송된 이미지 데이터가 포함되어 있습니다.
        // 구독자는 메시지를 받을 때마다 image_callback 함수를 호출합니다.
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", // 구독할 토픽의 이름입니다.
            10,                  // 메시지 큐의 크기입니다. 너무 많은 메시지가 한 번에 들어오면 일부가 무시될 수 있습니다.
            std::bind(&LineTracingRobot::image_callback, this, std::placeholders::_1)
            // image_callback 함수와 연결(bind)하여, 메시지를 받을 때마다 호출합니다.
            // std::bind는 멤버 함수를 호출하기 위해 사용됩니다.
        );

        // 로봇에 속도 명령을 보내기 위한 퍼블리셔를 생성합니다.
        // "/cmd_vel" 토픽에 Twist 메시지를 퍼블리시합니다.
        // Twist 메시지는 로봇의 선형 속도(linear)와 회전 속도(angular)를 나타냅니다.
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 타이머를 설정하여 초기 전진 동작과 정지 대기 동작을 제어합니다.
        // 이 타이머는 100ms마다 타이머 콜백을 호출합니다.
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LineTracingRobot::timer_callback, this));
    }

private:
    // 타이머 콜백 함수: 초기 전진 후 정지 대기를 하고, 라인트레이싱을 활성화합니다.
    void timer_callback()
    {
        if (!initial_move_done_) // 초기 전진이 완료되지 않은 경우
        {
            auto command = geometry_msgs::msg::Twist();
            command.linear.x = 0.1;               // 살짝 전진하도록 설정합니다.
            cmd_vel_publisher_->publish(command); // 명령을 퍼블리시하여 로봇을 이동시킵니다.

            initial_move_duration_ += 0.1;     // 타이머가 100ms 간격으로 호출되므로 0.1초씩 더합니다.
            if (initial_move_duration_ >= 0.5) // 전진 시간이 0.5초 이상이 되면
            {
                initial_move_done_ = true;    // 초기 전진이 완료되었음을 표시합니다.
                waiting_after_move_ = true;   // 정지 대기 상태로 전환합니다.
                initial_move_duration_ = 0.0; // 대기 시간 추적을 위해 초기화합니다.
            }
        }
        else if (waiting_after_move_) // 초기 전진 후 정지 대기 상태인 경우
        {
            auto command = geometry_msgs::msg::Twist();
            command.linear.x = 0.0;               // 로봇을 정지시킵니다.
            cmd_vel_publisher_->publish(command); // 명령을 퍼블리시하여 로봇을 정지시킵니다.

            initial_move_duration_ += 0.1;     // 타이머가 100ms 간격으로 호출되므로 0.1초씩 더합니다.
            if (initial_move_duration_ >= 1.5) // 대기 시간이 1.5초 이상이 되면
            {
                waiting_after_move_ = false; // 대기 상태를 해제합니다.
                timer_->cancel();            // 타이머를 중지합니다.
            }
        }
    }

    // 이미지 콜백 함수: 카메라에서 이미지를 수신할 때마다 호출됩니다.
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // QR 코드가 아직 인식되지 않은 경우 QR 코드를 먼저 인식합니다.
        if (!qr_code_detected_)
        {
            start_point_ = detect_qr_code(msg); // QR 코드를 인식하여 출발지를 설정합니다.
            if (start_point_ > 0)
            {
                qr_code_detected_ = true; // QR 코드가 인식되었음을 표시합니다.
                RCLCPP_INFO(this->get_logger(), "QR code detected: START%d", start_point_);
            }
            return; // QR 코드가 인식될 때까지 대기합니다.
        }

        if (!initial_move_done_ || waiting_after_move_)
            return; // 초기 전진 및 대기 상태가 완료되지 않으면 라인트레이싱을 수행하지 않습니다.

        // 이미지에서 검정색 선을 감지하는 로직
        bool left_detected = false;
        bool right_detected = false;
        bool center_detected = false;

        // 이미지 데이터를 순회하며 검정색(예: RGB 값이 모두 낮은 값) 픽셀을 찾습니다.
        for (size_t i = 0; i < msg->data.size(); i += 3)
        {
            int r = msg->data[i];
            int g = msg->data[i + 1];
            int b = msg->data[i + 2];

            if (r < 50 && g < 50 && b < 50) // 검정색 픽셀 감지
            {
                if (i % msg->width < msg->width / 3)
                    left_detected = true; // 이미지의 왼쪽 영역에서 검정색 감지
                else if (i % msg->width > 2 * msg->width / 3)
                    right_detected = true; // 이미지의 오른쪽 영역에서 검정색 감지
                else
                    center_detected = true; // 이미지의 중앙에서 검정색 감지
            }
        }

        auto command = geometry_msgs::msg::Twist();

        // 출발지에 따른 경로 선택 로직
        if (start_point_ == 1) // 1번 출발지에서 출발한 경우
        {
            if (center_detected || right_detected)
                command.linear.x = 0.1; // 중앙 또는 오른쪽에 검정색 선이 있으면 전진
            else
                command.linear.x = 0.0; // 검정색 선이 없으면 정지
        }
        else if (start_point_ == 2) // 2번 출발지에서 출발한 경우
        {
            if (center_detected || left_detected)
                command.linear.x = 0.1; // 중앙 또는 왼쪽에 검정색 선이 있으면 전진
            else
                command.linear.x = 0.0; // 검정색 선이 없으면 정지
        }

        cmd_vel_publisher_->publish(command); // 명령을 퍼블리시하여 로봇을 이동시키거나 정지시킵니다.
    }

    // QR 코드를 인식하는 함수
    int detect_qr_code(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        zbar::ImageScanner scanner;                                      // zbar 라이브러리의 이미지 스캐너를 사용합니다.
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1); // QR 코드 인식을 활성화합니다.

        // ROS 이미지 데이터를 zbar에서 처리할 수 있는 형식으로 변환
        zbar::Image image(msg->width, msg->height, "Y800", (void *)msg->data.data(), msg->data.size());
        scanner.scan(image); // 이미지를 스캔하여 QR 코드를 탐지

        // QR 코드 데이터를 확인하고, 해당하는 출발지 번호를 반환
        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            if (symbol->get_data() == "START1")
                return 1; // "START1" QR 코드를 인식한 경우
            if (symbol->get_data() == "START2")
                return 2; // "START2" QR 코드를 인식한 경우
        }
        return 0; // QR 코드가 인식되지 않으면 0을 반환
    }

    bool initial_move_done_;             // 초기 전진 동작이 완료되었는지 여부를 저장하는 변수입니다.
    bool waiting_after_move_;            // 초기 전진 후 대기 상태인지 여부를 저장하는 변수입니다.
    bool qr_code_detected_;              // QR 코드가 인식되었는지를 추적하는 변수입니다.
    double initial_move_duration_ = 0.0; // 초기 전진 및 대기 시간이 얼마나 지났는지를 저장하는 변수입니다.
    int start_point_;                    // 인식된 출발지를 저장하는 변수입니다.

    // 카메라에서 이미지를 수신하기 위한 ROS2 구독자입니다.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    // 로봇에 속도 명령을 보내기 위한 ROS2 퍼블리셔입니다.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // 초기 전진과 대기 동작을 위한 타이머입니다.
    rclcpp::TimerBase::SharedPtr timer_;
};

// main 함수는 C++ 프로그램의 진입점입니다.
// ROS2 시스템을 초기화하고, 노드를 생성하여 콜백 처리를 시작합니다.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                           // ROS2 시스템을 초기화합니다. 이 함수는 ROS2 애플리케이션의 필수적인 초기화 단계입니다.
    rclcpp::spin(std::make_shared<LineTracingRobot>()); // LineTracingRobot 노드를 생성하고 실행을 유지합니다.
    // rclcpp::spin 함수는 노드가 종료될 때까지 지속적으로 실행되며, 콜백 함수들이 호출될 수 있도록 합니다.
    rclcpp::shutdown(); // 노드가 종료되면 ROS2 시스템을 정리하고 종료합니다.
    return 0;           // 프로그램을 정상 종료합니다.
}

