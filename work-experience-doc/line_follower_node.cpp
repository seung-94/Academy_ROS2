#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LineFollower : public rclcpp::Node
{
public:
    LineFollower() : Node("line_follower")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&LineFollower::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 이미지 데이터를 처리하여 라인을 추적하는 로직을 작성
        int width = msg->width;
        int height = msg->height;
        const auto &data = msg->data;

        // 간단한 예제: 이미지의 중앙 부분의 색상을 확인하여 라인을 추적
        int sum_x = 0;
        int count = 0;
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                int index = (i * width + j) * 3; // Assuming 3 bytes per pixel (RGB)
                int r = data[index];
                int g = data[index + 1];
                int b = data[index + 2];

                // 간단한 이진화: 특정 임계값 아래의 픽셀을 검정색으로 간주
                if (r < 50 && g < 50 && b < 50)
                {
                    sum_x += j;
                    count++;
                }
            }
        }

        geometry_msgs::msg::Twist cmd_msg;

        if (count > 0)
        {
            int avg_x = sum_x / count;
            int error = avg_x - width / 2;

            cmd_msg.linear.x = 0.2; // 전진 속도
            cmd_msg.angular.z = -error * 0.005; // 회전 속도 (P 제어)
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.2; // 라인을 찾지 못했을 때 회전
        }

        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollower>());
    rclcpp::shutdown();
    return 0;
}
