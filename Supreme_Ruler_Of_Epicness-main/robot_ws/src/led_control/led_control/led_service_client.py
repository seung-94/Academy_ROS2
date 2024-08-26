""" 
    led_service_client.py (LED Service Client)
    역할: 이 노드는 LED 제어 서비스를 호출하여 LED를 켜거나 끌수 있도록 제어하는 클라이언트 역할을 합니다. 
    동작에 필요한 이유: 로봇의 상태에 따라 LED를 제어하여 사용자에게 시각적인 피드백을 제공할 수 있습니다.
    사용 시나리오: 특정 이벤트 발생 시 LED를 켜거나 끄기 위해 이 노드를 사용합니다. 
    실행 필요성: LED 제어가 필요할 때 이 노드를 실행하여 LED를 켜거나 끌 수 있습니다.
"""

import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from std_srvs.srv import SetBool  # SetBool 서비스 타입

# LedClient 클래스: LED 제어 서비스 클라이언트 노드
class LedClient(Node):
    def __init__(self):
        super().__init__('led_client')  # 노드 이름 설정
        self.client = self.create_client(SetBool, 'toggle_led')  # 서비스 클라이언트 생성
        while not self.client.wait_for_service(timeout_sec=1.0):  # 서비스 서버가 준비될 때까지 대기
            self.get_logger().info('Service not available, waiting...')

        self.turn_led_on()  # LED 켜기

    def turn_led_on(self):
        request = SetBool.Request()  # 서비스 요청 메시지 생성
        request.data = True  # LED를 켜기 위해 True로 설정
        self.future = self.client.call_async(request)  # 비동기 서비스 호출
        self.future.add_done_callback(self.response_callback)  # 서비스 호출 완료 후 콜백 함수 설정

    def response_callback(self, future):
        try:
            response = future.result()  # 서비스 응답 가져오기
            self.get_logger().info(f"LED state: {response.message}")  # 응답 메시지 출력
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")  # 오류 발생 시 로그 출력

# 메인 함수: 노드를 초기화하고 실행
def main(args=None):
    rclpy.init(args=args)
    node = LedClient()  # 노드 생성
    rclpy.spin(node)  # 노드 실행 및 서비스 요청 대기
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()

""" 
    질문: 왜 async_send_request를 사용하나요?
    답변: 비동기 방식으로 요청을 보내면 LED 상태를 제어하는 동안 다른 작업을 방해하지 않고 프로그램이 계속 실행될 수 있습니다. 특히 로봇이 다양한 작업을 동시에 수행해야 할 때 유용합니다.
         더 자세히, async_send_request를 사용하여 비동기적으로 LED 제어 요청을 보내는 이유는 로봇의 다른 작업을 중단하지 않고도 LED를 제어할 수 있도록 하여,
         전체 시스템의 실시간성과 응답성을 유지하기 위함입니다.

    질문: 왜 SetBool 서비스를 사용하나요?
    답변: LED의 상태는 간단한 ON/OFF로 표현될 수 있습니다. 따라서 SetBool 서비스는 이진 상태를 제어하는 데 적합합니다.
"""