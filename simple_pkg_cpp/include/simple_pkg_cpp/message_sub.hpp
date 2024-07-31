#ifndef MESSAGESUBSCRIBER_HPP
#define MESSAGESUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono_literals;

class MessageSubscriber : public rclcpp::Node
{
public:
    MessageSubscriber();

private:
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr _sub;
    void sub_message_msg(const std_msgs::msg::Header::SharedPtr msg);
};

#endif // MESSAGESUBSCRIBER_HPP