#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class LineFollower : public rclcpp::Node
{
public:
    LineFollower()
    : Node("line_follower")
    {
        // Publisher for robot velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribers for line sensors
        left_sensor_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "left_sensor", 10, std::bind(&LineFollower::left_sensor_callback, this, std::placeholders::_1));
        center_sensor_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "center_sensor", 10, std::bind(&LineFollower::center_sensor_callback, this, std::placeholders::_1));
        right_sensor_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "right_sensor", 10, std::bind(&LineFollower::right_sensor_callback, this, std::placeholders::_1));
        
        // Initial sensor states
        left_sensor_ = false;
        center_sensor_ = false;
        right_sensor_ = false;

        // Timer to run the control loop
        timer_ = this->create_wall_timer(
            100ms, std::bind(&LineFollower::control_loop, this));
    }

private:
    void left_sensor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        left_sensor_ = msg->data;
    }

    void center_sensor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        center_sensor_ = msg->data;
    }

    void right_sensor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        right_sensor_ = msg->data;
    }

    void control_loop()
    {
        auto message = geometry_msgs::msg::Twist();
        
        if (center_sensor_)
        {
            // Move forward
            message.linear.x = 0.5;
            message.angular.z = 0.0;
        }
        else if (left_sensor_ && !right_sensor_)
        {
            // Turn right
            message.linear.x = 0.0;
            message.angular.z = -0.5;
        }
        else if (right_sensor_ && !left_sensor_)
        {
            // Turn left
            message.linear.x = 0.0;
            message.angular.z = 0.5;
        }
        else
        {
            // Stop or handle no line detected case
            message.linear.x = 0.0;
            message.angular.z = 0.0;
        }

        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_sensor_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr center_sensor_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_sensor_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool left_sensor_;
    bool center_sensor_;
    bool right_sensor_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollower>());
    rclcpp::shutdown();
    return 0;
}
