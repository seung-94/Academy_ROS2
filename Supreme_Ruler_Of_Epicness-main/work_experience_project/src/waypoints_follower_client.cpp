/*
    waypoints_follower_client.cpp (Waypoints Follower Client)
    역할: 이 노드는 미리 정의된 경로를 따라가는 액션 클라이언트 역할을 하며, 로봇이 특정 경로를 따르도록 명령을 전달합니다.
    동작에 필요한 이유: 로봇이 여러 경유지를 거쳐 이동해야 할 때 이 노드를 통해 경로를 전달하여 로봇이 정확한 경로를 따라갈 수 있게 합니다.
    사용 시나리오: 특정 작업 공간을 순찰하거나 지정된 포인트를 방문해야 할 때 이 노드를 사용하여 경로를 설정하고 로봇을 이동시킵니다.
    실행 필요성: 경로를 따라 로봇을 이동시키려면 반드시 이 노드를 실행해야 합니다. 이 노드가 없으면 경로에 따라 로봇을 제어할 수 없습니다.
*/

#include "rclcpp/rclcpp.hpp"                       // ROS2 기본 기능을 위한 헤더 파일
#include "geometry_msgs/msg/pose_stamped.hpp"      // PoseStamped 메시지 사용을 위한 헤더 파일
#include "nav2_msgs/action/follow_waypoints.hpp"   // FollowWaypoints 액션 사용을 위한 헤더 파일
#include "std_srvs/srv/set_bool.hpp"               // SetBool 서비스 사용을 위한 헤더 파일
#include "rclcpp_action/rclcpp_action.hpp"         // ROS2 액션 클라이언트를 위한 헤더 파일
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Pose와 쿼터니언 변환을 위한 헤더 파일
#include "tf2/LinearMath/Quaternion.h"             // 쿼터니언(회전) 연산을 위한 헤더 파일

// WaypointsFollower 클래스: 로봇이 설정된 경로를 따라가도록 하는 액션 클라이언트 노드
class WaypointsFollower : public rclcpp::Node
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    // 생성자: 노드 이름을 지정하고 액션 클라이언트 및 서비스 클라이언트 초기화
    WaypointsFollower(const rclcpp::NodeOptions &options) : Node("waypoints_follower_client", options)
    {
        // "follow_waypoints" 액션 서버와 통신할 클라이언트 생성
        _action_client = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

        // LED 제어를 위한 서비스 클라이언트 생성
        _led_client = this->create_client<std_srvs::srv::SetBool>("toggle_led");

        // 프로그램 시작 시 LED 켜기
        toggle_led(true);

        // 주기적으로 목표를 전송하기 위한 타이머 초기화
        _timer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&WaypointsFollower::send_goal, this));
    }

private:
    // LED 제어 함수
    void toggle_led(bool state)
    {
        // 서비스 호출을 위한 요청 메시지 생성
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = state;

        // 서비스 준비 대기 시간 연장
        if (!_led_client->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(this->get_logger(), "LED service is not available.");
            return;
        }

        // 비동기 서비스 호출
        _led_client->async_send_request(request, [this, state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
                                        {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "LED %s", state ? "on" : "off");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to toggle LED: %s", response->message.c_str());
        } });
    }

    // 타이머 콜백 함수: 경유지 목표를 전송하는 함수
    void send_goal()
    {
        _timer->cancel(); // 타이머를 취소하여 한 번만 실행되도록 설정

        // 액션 서버가 사용 가능한지 확인
        if (!_action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            rclcpp::shutdown(); // 액션 서버가 없으면 노드를 종료
            return;
        }

        // 경로를 정의하기 위해 목표 메시지를 생성
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.44, -0.158, 1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(1.43, 0.353, 3.14));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.122, 0.465, -1.57));
        goal_msg.poses.push_back(get_pose_from_xy_theta(0.0, 0.0, 0.0)); // 원점(0,0)으로 이동

        RCLCPP_INFO(get_logger(), "Sending goal request");

        // 목표 전송 옵션 설정: 응답, 피드백, 결과 콜백 함수 정의
        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&WaypointsFollower::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&WaypointsFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&WaypointsFollower::result_callback, this, std::placeholders::_1);

        // 목표를 비동기적으로 전송
        _action_client->async_send_goal(goal_msg, send_goal_options);
    }

    // 좌표와 각도를 사용해 PoseStamped 메시지를 생성하는 함수
    geometry_msgs::msg::PoseStamped get_pose_from_xy_theta(const float &x, const float &y, const float &theta)
    {
        // PoseStamped 메시지 초기화
        geometry_msgs::msg::PoseStamped pose;
        tf2::Quaternion q;     // 회전을 표현하기 위한 쿼터니언 객체 생성
        q.setRPY(0, 0, theta); // 회전을 Yaw 값으로 설정 (Roll과 Pitch는 0으로 설정)

        pose.header.frame_id = "map";                 // 좌표계 기준을 "map"으로 설정
        pose.header.stamp = this->get_clock()->now(); // 현재 시간을 타임스탬프로 설정
        pose.pose.position.x = x;                     // x 좌표 설정
        pose.pose.position.y = y;                     // y 좌표 설정
        pose.pose.position.z = 0.0;                   // z 좌표는 사용하지 않음
        pose.pose.orientation = tf2::toMsg(q);        // 쿼터니언을 메시지로 변환하여 설정

        return pose;
    }

    // 목표 응답 콜백 함수: 목표가 수락되었는지 여부를 확인
    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server"); // 목표가 서버에 의해 거부된 경우
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result"); // 목표가 수락된 경우
        }
    }

    // 피드백 콜백 함수: 현재 경유지 정보를 출력
    void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                           const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", feedback->current_waypoint); // 현재 경유지 정보 출력
    }

    // 결과 콜백 함수: 목표 달성 결과를 처리, 원점에 도착했을 경우 프로그램 종료 및 LED 끄기
    void result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded!"); // 목표가 성공적으로 달성된 경우

            // 최종 도착지(원점)에 도착했으므로 LED 끄기
            toggle_led(false);

            rclcpp::shutdown(); // 프로그램 종료
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(get_logger(), "Goal was aborted"); // 목표가 중단된 경우
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal was canceled"); // 목표가 취소된 경우
            break;
        default:
            RCLCPP_INFO(get_logger(), "Unknown goal result code"); // 알 수 없는 결과 코드
            break;
        }
    }

    // 액션 클라이언트, 타이머, 서비스 클라이언트 선언
    rclcpp_action::Client<FollowWaypoints>::SharedPtr _action_client; // FollowWaypoints 액션 클라이언트
    rclcpp::TimerBase::SharedPtr _timer;                              // 주기적으로 목표를 전송하는 타이머
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _led_client;    // LED 제어 서비스 클라이언트
};

int main(int argc, char *argv[])
{
    // ROS2 시스템 초기화
    rclcpp::init(argc, argv);
    // WaypointsFollower 노드 실행
    auto node = std::make_shared<WaypointsFollower>(rclcpp::NodeOptions());
    rclcpp::spin(node); // 노드 실행 및 콜백 처리
    // ROS2 시스템 종료
    rclcpp::shutdown();
    return 0;
}
