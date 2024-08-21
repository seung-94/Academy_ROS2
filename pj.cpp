#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32.hpp>

class LineFollowingRobot : public rclcpp::Node
{
public:
    LineFollowingRobot(const std::string &robot_name)
        : Node(robot_name)
    {
        // 카메라 토픽 구독
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LineFollowingRobot::image_callback, this, std::placeholders::_1));

        // 모터 제어를 위한 출판자 초기화 (예시 토픽 이름)
        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("/left_motor_speed", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("/right_motor_speed", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 이미지 메시지를 OpenCV 이미지로 변환
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 그레이스케일로 변환
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 이진화
        cv::Mat binary;
        cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY_INV);

        // 관심 영역(ROI) 설정
        int height = binary.rows;
        int width = binary.cols;
        cv::Mat roi = binary(cv::Rect(0, height / 2, width, height / 2));

        // 윤곽선 검출
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // 가장 큰 윤곽선 찾기
            auto max_contour = std::max_element(contours.begin(), contours.end(),
             [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
              { return cv::contourArea(a) < cv::contourArea(b); });

            // 무게중심 계산
            cv::Moments M = cv::moments(*max_contour);
            int cx = static_cast<int>(M.m10 / M.m00);

            // 중심선과 프레임의 중심 비교
            int error = cx - width / 2;

            // 기본 주행 명령
            float left_speed = 50.0;
            float right_speed = 50.0;

            // 방향 조정
            if (error > 10)  // 허용 오차를 두고 오른쪽으로 돌도록 설정
            {
                right_speed = 40.0;
            }
            else if (error < -10)  // 허용 오차를 두고 왼쪽으로 돌도록 설정
            {
                left_speed = 40.0;
            }

            // 모터 속도 출판
            publish_motor_speed(left_speed, right_speed);
        }
        else
        {
            // 선을 잃어버렸을 경우 모터 정지
            publish_motor_speed(0.0, 0.0);
        }
    }

    void publish_motor_speed(float left_speed, float right_speed)
    {
        auto left_msg = std_msgs::msg::Float32();
        auto right_msg = std_msgs::msg::Float32();

        left_msg.data = left_speed;
        right_msg.data = right_speed;

        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_motor_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 단일 로봇 객체 생성
    auto robot = std::make_shared<LineFollowingRobot>("line_following_robot");

    // 노드 실행
    rclcpp::spin(robot);

    rclcpp::shutdown();
    return 0;
}
