import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드의 기본 클래스
from std_srvs.srv import SetBool  # SetBool 서비스 타입을 사용하기 위한 모듈

# EmergencyStopClient 클래스 정의: 비상정지 서비스를 호출하는 클라이언트 노드
class EmergencyStopClient(Node):
    """ 
        __init__ 메서드:
        - self.create_client를 통해 emergency_stop이라는 이름의 SetBool 타입 서비스를 호출할 수 있는 클라이언트를 생성합니다.
        - 서비스가 활성화될 때까지 1초마다 대기하며, 대기 중임을 로그로 출력합니다.
    """
    def __init__(self):
        super().__init__('emergency_stop_client')  # 노드 이름을 'emergency_stop_client'로 설정하여 초기화
        self.client = self.create_client(SetBool, 'emergency_stop')  # 'emergency_stop' 서비스를 위한 클라이언트 생성
        
        # 서비스 서버가 시작될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency stop service...')  # 서비스가 준비될 때까지 대기 중임을 로그로 출력

    """ 
        send_emergency_stop 메서드:
        - True 또는 False 값을 받아 비상정지를 활성화하거나 해제하는 서비스 요청을 생성합니다.
        - 서비스는 비동기적으로 호출되며, 응답이 도착하면 handle_response 콜백 함수가 실행됩니다.
    """
    # 비상정지 명령을 전송하는 함수
    def send_emergency_stop(self, stop):
        request = SetBool.Request()  # SetBool 서비스 요청 객체 생성
        request.data = stop  # 요청 데이터로 True(정지) 또는 False(재개)를 설정
        future = self.client.call_async(request)  # 비동기적으로 서비스 호출
        future.add_done_callback(self.handle_response)  # 서비스 호출 완료 시 콜백 함수 등록
        
    """ 
        handle_response 메서드:
        - 서비스 응답을 받아 성공 여부를 로그로 출력합니다.
        - 예외가 발생하면 에러 메시지를 출력합니다.
    """
    # 서비스 호출에 대한 응답을 처리하는 콜백 함수
    def handle_response(self, future):
        try:
            response = future.result()  # 서비스 응답 결과를 가져옴
            self.get_logger().info(f'Response: {response.message}')  # 서비스 응답 메시지를 로그로 출력
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')  # 서비스 호출 실패 시 에러 메시지 출력

# 메인 함수: 노드를 초기화하고 실행
def main(args=None):
    rclpy.init(args=args)  # rclpy 초기화
    node = EmergencyStopClient()  # EmergencyStopClient 노드 인스턴스 생성
    
    # 비상정지 활성화 (로봇 정지)
    node.send_emergency_stop(True)
    
    # 비상정지 해제 (로봇 재개)
    # node.send_emergency_stop(False)
    
    rclpy.spin(node)  # 노드를 계속 실행하여 서비스 요청을 처리함
    rclpy.shutdown()  # rclpy 종료

if __name__ == '__main__':
    main()  # 메인 함수 실행

""" 
비상정지 활성화 (로봇 정지):
터미널에서 다음 명령어를 실행하여 로봇을 즉시 멈출 수 있습니다:
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"

비상정지 해제 (로봇 재개):
터미널에서 다음 명령어를 실행하여 로봇의 동작을 재개할 수 있습니다:
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"
 """
