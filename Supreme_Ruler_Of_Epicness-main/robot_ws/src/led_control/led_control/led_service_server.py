""" 
    led_service_server.py (LED Service Server)
    역할: 이 노드는 GPIO 핀을 사용하여 LED를 제어하는 서비스 서버입니다. 클라이언트로부터 LED를 켜거나 끄라는 요청을 받아 처리합니다.
    동작에 필요한 이유: 로봇의 상태를 시각적으로 표시하기 위해 LED를 제어하는 것이 필요합니다. 예를 들어, 특정 경유지에 도달했을 때 LED를 켜거나 끌 수 있습니다.
    사용 시나리오: LED를 켜거나 끄기 위해 이 노드를 사용합니다. 예를 들어, 로봇이 작업을 완료했음을 표시할 때 LED를 깜빡이도록 할 수 있습니다.
    실행 필요성: LED를 제어하려면 이 노드를 실행해야 합니다.
"""

import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from std_srvs.srv import SetBool  # SetBool 서비스 타입
import RPi.GPIO as g  # GPIO 제어를 위한 라이브러리

# GPIO 초기 설정
g.setmode(g.BCM)
g.setup(21, g.OUT)  # 21번 핀을 출력으로 설정

# Hbmove 클래스: LED를 제어하는 서비스 서버 노드
class Hbmove(Node):
    def __init__(self):
        super().__init__("led_service_server")  # 노드 이름 설정
        self.srv = self.create_service(SetBool, 'toggle_led', self.toggle_led_callback)  # 서비스 서버 생성

    def toggle_led_callback(self, request, response):
        # 요청 데이터에서 bool 값을 추출하여 LED 상태 결정
        str_data = str(request.data).lower()
        self.get_logger().info(f'Received request: {str_data} (converted from: {request.data}, type: {type(request.data)})')

        # 요청에 따라 LED 상태를 설정
        led_state = str_data not in ['false', '0']
        g.output(21, led_state)  # GPIO 핀을 통해 LED 켜기/끄기

        # 응답 메시지 설정
        response.success = True
        response.message = "LED is ON" if led_state else "LED is OFF"
        self.get_logger().info(f'LED should now be {"ON" if led_state else "OFF"}')

        return response  # 응답 반환

# 메인 함수: 노드를 초기화하고 실행
def main():
    rclpy.init()
    node = Hbmove()  # 노드 생성
    try:
        rclpy.spin(node)  # 노드를 실행하여 서비스 요청 대기
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        g.cleanup(21)  # GPIO 핀 정리
        node.destroy_node()
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()

""" 
    질문: 왜 RPi.GPIO를 사용하나요?
    답변: RPi.GPIO는 라즈베리파이의 GPIO 핀을 제어하는 데 사용되는 라이브러리로, LED와 같은 외부 장치의 제어에 적합합니다.
         더욱 자세히 설명을 부가하면, RPi.GPIO를 사용하여 직접적으로 라즈베리파이의 GPIO 핀을 제어하는 이유는 하드웨어 레벨에서 LED와 같은 외부 장치의 제어가 가능하기 때문입니다.
         이는 소프트웨어적 제어보다 즉각적이며, 물리적 장치와의 상호작용을 직접적으로 다룰 수 있는 장점이 있습니다.
    (*중복)질문: 왜 SetBool 서비스를 사용하나요?
    답변: LED의 상태는 ON/OFF로 제어할 수 있으므로, 불린 값을 사용하여 간단하게 제어하기 위해 SetBool 서비스를 사용합니다.
"""