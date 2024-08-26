"""
    emergency_stop_service_server.py (Emergency Stop Service Server)
    역할: 이 노드는 로봇의 동작을 멈추는 비상정지 기능을 수행하는 서비스 서버입니다. 클라이언트로부터 비상정지 요청을 받아 처리합니다.
    동작에 필요한 이유: 로봇의 안전한 운용을 위해 필요합니다. 예상치 못한 상황에서 로봇을 즉시 정지시킬 수 있습니다.
    사용 시나리오: 로봇이 위험한 상황에 처했을 때, 이 노드를 통해 즉시 정지 명령을 내릴 수 있습니다.
    실행 필요성: 로봇 운용 중 비상 상황이 발생할 수 있으므로 항상 실행할 필요가 있습니다.
"""

import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from std_srvs.srv import SetBool  # SetBool 서비스 타입

class EmergencyStopServer(Node):
    def __init__(self):
        super().__init__('emergency_stop_server')  # 노드 이름 설정
        self.srv = self.create_service(SetBool, 'emergency_stop', self.handle_emergency_stop)  # 서비스 서버 생성

    def handle_emergency_stop(self, request, response):
        if request.data:
            self.get_logger().info("Emergency Stop Triggered!")
            # 로봇 동작 중지 로직을 여기 추가 (예: 모터 전원 끄기, 명령 중지 등)
            response.success = True
            response.message = "Robot stopped."
        else:
            self.get_logger().info("Emergency Stop Released")
            # 로봇 동작 재개 로직을 여기 추가 (예: 모터 전원 켜기, 명령 재개 등)
            response.success = True
            response.message = "Robot resumed."

        return response

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopServer()  # 노드 생성
    rclpy.spin(node)  # 노드 실행 및 서비스 요청 대기
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()

"""
    질문: 왜 SetBool 서비스를 사용하나요?
    답변: SetBool 서비스는 True 또는 False의 간단한 값을 전송하여 로봇의 상태를 제어하는 데 적합합니다. 비상정지와 같은 이진 상태를 다루기 위해 적합한 서비스입니다.
"""
