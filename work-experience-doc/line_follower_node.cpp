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

*/
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
        // 이미지 데이터를 처리하여 검은 선을 추적하는 로직을 작성
        int width = msg->width;
        int height = msg->height;
        const auto &data = msg->data;

        int sum_x = 0;
        int count = 0;

        // 이미지의 하단 부분만 처리하여 선을 감지 (중앙부 하단의 세로 1/4 부분만 처리)
        int row_start = height * 3 / 4;
        for (int i = row_start; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                int index = (i * width + j) * 3; // Assuming 3 bytes per pixel (RGB)
                int r = data[index];
                int g = data[index + 1];
                int b = data[index + 2];

                // 검은색 선 검출: RGB 값이 모두 낮은 픽셀을 검은 선으로 간주
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
            int avg_x = sum_x / count; // 검출된 검은 선의 평균 x 좌표 계산
            int error = avg_x - width / 2; // 이미지 중앙으로부터의 오차 계산

            cmd_msg.linear.x = 0.2; // 전진 속도
            cmd_msg.angular.z = -error * 0.005; // 회전 속도 (P 제어)
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.2; // 검은 선을 찾지 못했을 때 회전하여 선을 탐색
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

/*

// 여러대 제어

*/
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>

class LineFollower : public rclcpp::Node
{
public:
    LineFollower(const std::string &robot_namespace) : Node("line_follower_" + robot_namespace)
    {
        std::string image_topic = "/" + robot_namespace + "/camera/image_raw";
        std::string cmd_vel_topic = "/" + robot_namespace + "/cmd_vel";

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&LineFollower::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 이미지 데이터를 처리하여 검은 선을 추적하는 로직을 작성
        int width = msg->width;
        int height = msg->height;
        const auto &data = msg->data;

        int sum_x = 0;
        int count = 0;

        // 이미지의 하단 부분만 처리하여 선을 감지 (중앙부 하단의 세로 1/4 부분만 처리)
        int row_start = height * 3 / 4;
        for (int i = row_start; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                int index = (i * width + j) * 3; // Assuming 3 bytes per pixel (RGB)
                int r = data[index];
                int g = data[index + 1];
                int b = data[index + 2];

                // 검은색 선 검출: RGB 값이 모두 낮은 픽셀을 검은 선으로 간주
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
            int avg_x = sum_x / count; // 검출된 검은 선의 평균 x 좌표 계산
            int error = avg_x - width / 2; // 이미지 중앙으로부터의 오차 계산

            cmd_msg.linear.x = 0.2; // 전진 속도
            cmd_msg.angular.z = -error * 0.005; // 회전 속도 (P 제어)
        }
        else
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.2; // 검은 선을 찾지 못했을 때 회전하여 선을 탐색
        }

        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "You must specify a robot namespace.");
        return 1;
    }

    std::string robot_namespace = argv[1];
    rclcpp::spin(std::make_shared<LineFollower>(robot_namespace));
    rclcpp::shutdown();
    return 0;
}

/*

//bash

// ros2 run line_follower line_follower_node robot1
// ros2 run line_follower line_follower_node robot2
