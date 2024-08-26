from setuptools import find_packages, setup

package_name = 'led_and_emergency_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='LED and Emergency Control package for Raspberry Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service_server = led_and_emergency_control.led_service_server:main',  # LED 제어 서비스 서버
            'led_service_client = led_and_emergency_control.led_service_client:main',  # LED 제어 서비스 클라이언트
            'emergency_stop_service_server = led_and_emergency_control.emergency_stop_service_server:main',  # 비상정지 서비스 서버
            'emergency_stop_service_client = led_and_emergency_control.emergency_stop_service_client:main',  # 비상정지 서비스 클라이언트
        ],
    },
)

# 패키지 설정 주석:

# 이 파일은 Python 패키지 관리 도구인 setuptools를 사용하여 라즈베리 파이에서 실행될 'led_and_emergency_control' 패키지를 설정하는 데 사용됩니다.
# 이 패키지는 주로 LED 제어와 비상정지 기능을 구현하기 위한 것입니다.

# 주요 구성 요소:
# - packages: 패키지 내에 있는 모든 서브 패키지를 자동으로 탐색합니다. 'test' 디렉토리는 제외합니다.
# - data_files: ROS 2 환경에서 패키지를 찾기 위한 리소스 파일과 패키지 자체의 메타데이터 파일(package.xml)을 설정합니다.
# - install_requires: 이 패키지를 설치하기 위해 필요한 의존 패키지 목록입니다. 여기서는 기본적으로 setuptools만 요구됩니다.
# - entry_points: 패키지 내에서 실행 가능한 스크립트(즉, 노드)를 정의합니다. 각 스크립트는 ROS 2 노드로 실행됩니다.

# 콘솔 스크립트 설정:
# - 'led_service_server = led_and_emergency_control.led_service_server:main':
#   LED 제어 서비스 서버 노드를 실행하는 스크립트입니다. GPIO를 사용하여 LED를 제어하는 서버 역할을 합니다.
# - 'led_service_client = led_and_emergency_control.led_service_client:main':
#   LED 제어 서비스 클라이언트를 실행하는 스크립트입니다. 서버에 요청을 보내 LED를 켜거나 끄는 역할을 합니다.
# - 'emergency_stop_service_server = led_and_emergency_control.emergency_stop_service_server:main':
#   비상정지 기능을 제공하는 서비스 서버 노드를 실행하는 스크립트입니다. 로봇의 동작을 멈추거나 재개하는 요청을 처리합니다.
# - 'emergency_stop_service_client = led_and_emergency_control.emergency_stop_service_client:main':
#   비상정지 기능을 호출하는 클라이언트를 실행하는 스크립트입니다. 비상정지 명령을 서버에 보내 로봇의 동작을 제어합니다.

# 차별점:
# 이 설정은 ROS 2 기반의 라즈베리 파이 환경에서 효과적으로 LED를 제어하고, 로봇의 비상정지 기능을 관리할 수 있도록 최적화되어 있습니다.
# 또한, 비상정지와 같은 중요한 기능을 추가하여 더 높은 수준의 로봇 제어와 안전성을 제공합니다.
