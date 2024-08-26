from setuptools import find_packages, setup

package_name = 'led_control'

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
    description='LED control package for Raspberry Pi',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_service_server = led_control.led_service_server:main',  # LED 제어 서비스 서버
            'led_service_client = led_control.led_service_client:main',  # LED 제어 서비스 클라이언트
        ],
    },
)

# 패키지 설정 주석:

# 이 파일은 Python 패키지 관리 도구인 setuptools를 사용하여 라즈베리 파이에서 실행될 'led_control' 패키지를 설정하는 데 사용됩니다.

# 주요 구성 요소:
# - packages: 패키지 내에 있는 모든 서브 패키지를 자동으로 탐색합니다. 'test' 디렉토리는 제외합니다.
# - data_files: ROS 2 환경에서 패키지를 찾기 위한 리소스 파일과 패키지 자체의 메타데이터 파일(package.xml)을 설정합니다.
# - install_requires: 이 패키지를 설치하기 위해 필요한 의존 패키지 목록입니다. 여기서는 기본적으로 setuptools만 요구됩니다.
# - entry_points: 패키지 내에서 실행 가능한 스크립트(즉, 노드)를 정의합니다. 각 스크립트는 ROS 2 노드로 실행됩니다.

# 콘솔 스크립트 설정:
# - 'led_service_server = led_control.led_service_server:main':
#   LED 제어 서비스 서버 노드를 실행하는 스크립트입니다. GPIO를 사용하여 LED를 제어하는 서버 역할을 합니다.
# - 'led_service_client = led_control.led_service_client:main':
#   LED 제어 서비스 클라이언트를 실행하는 스크립트입니다. 서버에 요청을 보내 LED를 켜거나 끄는 역할을 합니다.
