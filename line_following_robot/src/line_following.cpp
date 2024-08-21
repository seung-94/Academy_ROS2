#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"

class LineFollowingRobot : public rclcpp::Node
{
public:
    LineFollowingRobot() : Node("line_follower")
    {
         auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        _sub = create_subscription<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", qos_profile, std::bind(&LineFollowingRobot::sub_img, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("tb3_0/cmd_vel", 10);
        cmd_pub1_ = this->create_publisher<geometry_msgs::msg::Twist>("tb3_1/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub1_;
    void sub_img(const sensor_msgs::msg::CompressedImage msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;

        // 그레이스케일로 변환
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // 양자화 적용
        int num_levels = 4;
        cv::Mat quantized_gray = gray / (256 / num_levels);  // 이미지 값 범위를 [0, num_levels)로 조정
        quantized_gray = quantized_gray * (256 / num_levels);  // 원래 범위로 복원

        // 이진화
        cv::Mat binary;
        cv::threshold(quantized_gray, binary, 127, 255, cv::THRESH_BINARY_INV);

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
            {return cv::contourArea(a) < cv::contourArea(b);});

            // 무게중심 계산
            cv::Moments M = cv::moments(*max_contour);
            int cx = static_cast<int>(M.m10 / M.m00);

            // 중심선과 프레임의 중심 비교
            int error = cx - width / 2;

            // 기본 주행 명령
            float linear_speed = 0.5;
            float angular_speed = 0.0;

            // 방향 조정
            if (error > 10)  // 허용 오차를 두고 오른쪽으로 돌도록 설정
            {
                angular_speed = -0.5; // 오른쪽으로 회전
            }
            else if (error < -10)  // 허용 오차를 두고 왼쪽으로 돌도록 설정
            {
                angular_speed = 0.5;  // 왼쪽으로 회전
            }

            // 두 대의 로봇에 대한 속도 출판
            publish_motor_speed(linear_speed, angular_speed);
        }
        else
        {
            // 선을 잃어버렸을 경우 모터 정지
            publish_motor_speed(0.0, 0.0);
        }
    }

    void publish_motor_speed(float linear_speed, float angular_speed)
    {
        auto cmd_msg_0 = geometry_msgs::msg::Twist();
        auto cmd_msg_1 = geometry_msgs::msg::Twist();

        // 첫 번째 로봇에 대한 명령 설정
        cmd_msg_0.linear.x = linear_speed;
        cmd_msg_0.angular.z = angular_speed;
        cmd_pub_->publish(cmd_msg_0);

        // 두 번째 로봇에 대한 명령 설정
        cmd_msg_1.linear.x = linear_speed;
        cmd_msg_1.angular.z = angular_speed;
        cmd_pub1_->publish(cmd_msg_1);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<LineFollowingRobot>();  // 로v봇 노드 객체 생성
    rclcpp::spin(robot);  // 노드 실행
    rclcpp::shutdown();   // 종료
    return 0;
}
