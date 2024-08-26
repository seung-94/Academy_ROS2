/*
    current_position_publisher.cpp (Current Position Publisher)
    역할: 이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시하는 역할을 합니다. 로봇의 위치 정보는 tf2 프레임워크를 통해 'base_link' 프레임과 'map' 프레임 간의 변환으로부터 가져옵니다.
         위치 정보는 "current_position" 토픽에 geometry_msgs::msg::PoseStamped 형식으로 퍼블리시됩니다.
    동작에 필요한 이유: 로봇이 이동하거나 특정 목표 위치로 이동하는 동안, 로봇의 현재 위치를 추적하고 이를 다른 노드들이 구독하여 사용할 수 있도록 정보를 제공하는 것이 필요합니다.
                    로봇의 현재 위치 정보는 경로 계획, 장애물 회피, 목표 지점 도달 여부 판단 등에 필수적입니다.
    사용 시나리오: 이 노드는 로봇의 현재 위치를 주기적으로 퍼블리시해야 하는 경우에 사용됩니다.
                예를 들어, 로봇의 위치 정보를 다른 노드들이 실시간으로 구독하여 경로 계획이나 위치 기반 작업을 수행할 때 유용합니다.
    실행 필요성: 로봇의 위치 정보가 필요하거나 기록되어야 할 때, 예를 들어 로봇이 특정 경로를 따라 이동할 때 경로 추적, 장애물 회피, 목표 도달 여부를 판단하기 위해 이 노드를 실행해야 합니다.
*/

#include "rclcpp/rclcpp.hpp"                       // ROS2 기본 기능을 위한 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp"      // PoseStamped 메시지 사용을 위한 헤더 파일
#include "tf2/LinearMath/Quaternion.h"             // 쿼터니언(회전) 연산을 위한 헤더 파일
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Pose와 쿼터니언 변환을 위한 헤더 파일
#include "tf2_ros/buffer.h"                        // tf2 버퍼를 사용하기 위한 헤더 파일
#include "tf2_ros/transform_listener.h"            // tf2 변환 리스너를 사용하기 위한 헤더 파일

// 현재 위치를 주기적으로 퍼블리시하는 노드
class CurrentPositionPublisher : public rclcpp::Node
{
public:
    // 생성자: 노드 이름을 지정하고 퍼블리셔와 타이머, tf2 버퍼와 리스너를 초기화
    CurrentPositionPublisher() : Node("current_position_publisher"),
                                 tf_buffer_(this->get_clock()), // tf 버퍼 초기화
                                 tf_listener_(tf_buffer_)       // tf 리스너 초기화
    {
        // 퍼블리셔 초기화: "current_position" 토픽에 PoseStamped 메시지 타입을 퍼블리시
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_position", 10);

        // 타이머 설정: 500ms마다 현재 위치를 퍼블리시
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&CurrentPositionPublisher::publish_current_position, this));
    }

private:
    // 타이머 콜백 함수: 주기적으로 현재 위치를 퍼블리시
    void publish_current_position()
    {
        auto current_position = get_current_position(); // 현재 위치를 가져옴
        publisher_->publish(current_position);          // 퍼블리셔를 통해 퍼블리시
    }

    // tf2를 이용하여 로봇의 실제 위치를 반환하는 함수
    geometry_msgs::msg::PoseStamped get_current_position()
    {
        geometry_msgs::msg::PoseStamped pose; // PoseStamped 메시지 생성

        try
        {
            // "base_link" (로봇 프레임)에서 "map" (월드 프레임)으로의 변환을 가져옴
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            // 변환된 위치와 회전 정보를 PoseStamped 메시지로 설정
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now(); // 현재 시간으로 타임스탬프 설정

            // 변환된 위치 설정
            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;

            // 변환된 회전 설정 (쿼터니언)
            pose.pose.orientation = transform_stamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }

        return pose; // 생성된 PoseStamped 메시지 반환
    }

    // 퍼블리셔와 타이머, tf2 버퍼와 리스너 선언
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // CurrentPositionPublisher 노드 실행
    rclcpp::spin(std::make_shared<CurrentPositionPublisher>());
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
