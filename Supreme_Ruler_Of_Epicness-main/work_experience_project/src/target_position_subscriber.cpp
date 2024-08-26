/*
    target_position_subscriber.cpp (Target Position Subscriber)
    역할: 이 노드는 "target_position" 토픽을 구독하여 로봇이 도달해야 할 목표 위치를 수신합니다. 수신된 위치는 geometry_msgs::msg::PoseStamped 메시지로 표현되며, 이 노드는 메시지의 내용을 콘솔에 출력합니다.
    동작에 필요한 이유: 이 노드는 목표 위치를 수신하여 로봇의 목표 위치를 기록하거나 처리하는 데 필요합니다. 다른 노드들이 로봇의 목표 위치를 인식하고 해당 위치로의 경로를 계산하는 데 사용될 수 있습니다.
    사용 시나리오: 이 노드는 목표 위치를 구독하여 특정 동작을 수행하는 경우에 사용됩니다. 목표 위치로 이동하거나 목표 위치 정보를 기반으로 로봇을 제어해야 할 때 필요합니다.
    실행 필요성: 목표 위치 정보를 받아서 이를 처리해야 하는 경우 실행합니다. 이 노드가 없으면 목표 위치 정보를 받을 수 없습니다.
*/

#include "rclcpp/rclcpp.hpp"                       // ROS2 기본 기능을 위한 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp"      // PoseStamped 메시지 사용을 위한 헤더 파일
#include "tf2/LinearMath/Quaternion.h"             // 쿼터니언(회전) 연산을 위한 헤더 파일
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // tf2::getYaw 함수 사용을 위한 헤더 파일

// 타겟 위치를 구독하여 처리하는 노드
class TargetPositionSubscriber : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 지정하고 구독자를 초기화
    TargetPositionSubscriber() : Node("target_position_subscriber")
    {
        // 구독자 초기화: "target_position" 토픽에서 PoseStamped 메시지를 구독
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_position", 10, std::bind(&TargetPositionSubscriber::target_position_callback, this, std::placeholders::_1));
    }

private:
    // 콜백 함수: 메시지를 수신했을 때 호출됨
    void target_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // tf2::Quaternion을 사용해 메시지에서 쿼터니언 회전 정보를 가져옴
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        double roll, pitch, yaw;
        // 쿼터니언을 롤, 피치, 요 값으로 변환
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 수신된 위치와 회전 정보를 로그로 출력
        RCLCPP_INFO(this->get_logger(), "Received target position: x=%.2f, y=%.2f, theta=%.2f",
                    msg->pose.position.x, msg->pose.position.y, yaw);
    }

    // 구독자 선언
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // TargetPositionSubscriber 노드 실행
    rclcpp::spin(std::make_shared<TargetPositionSubscriber>());
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
