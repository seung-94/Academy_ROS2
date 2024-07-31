#ifndef MESSAGEPUBLISHER_HPP
#define MESSAGEPUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <iostream>

using namespace std;
using namespace std::chrono_literals;

class MessagePublisher : public rclcpp::Node
{
public:
    MessagePublisher();

private:
    int _i;
    void publish_message_msg();
};

#endif // MESSAGEPUBLISHER_HPP