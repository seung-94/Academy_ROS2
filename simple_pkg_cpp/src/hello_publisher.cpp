#include "rclcpp/rclcpp.hpp"
#include <iostream>

using namespace std;

int main ()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shard<rclcpp::Node>("hello_rclcpp");
    rclcpp::spin(node)
    rclcpp::shutdown();
    cout << "Hello, World!" <<endl;
}