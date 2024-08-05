# Academy_ROS2

kdta-ros2

freshmea/kdta_ROS2

---------------------------------------------------
2024-07-23

1차시
- wmware 설치
- ubuntu 22.04 설치
- terminator 설치
- git 설치

2차시
- vscode 설치
- github 연동

3차시
- opencv 관련하여 point,size 등 기본 자료형 클래스 교육
- 주요 클래스 Point, Size, Rect, Scalar
---------------------------------------------------
2024-07-24

1차시
- 어제 수업내용 복습
- Mat클래스를 활용한 다양한 실습(1)

2차시
- Mat클래스를 활용한 다양한 실습(2)
- 행렬, resize 등의 Mat 클래스 활용

3차시
- Mat 클래스의 다양한 활용 실습
- 사진을 불러내거나 size를 조절 하거나 그림을 작성해서 이미지로 불러내는등의 활동을 함.
- Vec와 scalar 클래스 관련 실습
- 카메라와 동영상 파일 다루는 실습
- 다양한 그리기 실습 (line, rectangle, circle, ellipse, polylines, fillPoly, putText 함수)

오늘의 과제
- md 작성하는 방법 찾아보기
- make 기본 사용법 알아보기
- cmake 기본 사용법 알아보기
- cmake 총정리
---------------------------------------------------
2024-07-25

오전 수업
- 어제 수업내용 복습
- void onMouse 관련 실습 (mouse callback 함수 작성)
- 키보드나 마우스의 이벤트 사용실습 및 트랙바 사용실습
- FileStorage클래스 실습
- 데이터 파일 여러개 합쳐서 불러내기 실습
- 이미지의 밝기와 명암비 조절 하는 실습

오후 수업
- 오늘의 과제 코드 답 같이 짜기.
- 이미지의 밝기와 명암비 조절 하는 실습
- 히스토그램 펼치기 & 스트레칭 등의 실습
- blurr, Filter, shapen 등을 실습

오늘의 과제
- 마우스를 따라다니는 사각형이 이미지클릭 했을때의 RGB값을 출력하게 하는 코드 짜기
---------------------------------------------------
2024-07-26

오전 수업
- 어제 수업내용 복습
- affine 실습
- affine Rotation 실습
- 4개의 점을 지정해서 200 X 300의 이미지 만들기 실습
- houghline 실습

오후 수업
- adaptiveThreshold 실습
- cascade 실습
- morphology, threthold 실습
---------------------------------------------------
2024-07-29

오늘의 수업
- 박정석 강사님의 c++ 추가 강의
---------------------------------------------------
2024-07-30

오전 수업
- ROS2 시작
- ROS2 개념
  - 노드
  - 토픽
  - 메시지
  - 런처
  - 노드 통신
  - 노드 실행
- ROS2 설치
  - ROS2 humble 설치
  - ros2 humble documentation debian installation
  - 환경 설정 파일
    - .bashrc
      - source /opt/ros/humble/setup.bash
      - source ~/xxx_ws/install/local_setup.bash
      - 여러가지 alias 설정
      - export ROS_DOMAIN_ID 설정

오후 수업
- 패키지 작성
  - kdta_ws
    - simple_pkg_cpp
      - xml 수정
      - CMakeLists.txt 수정
      - 노드 추가
        - helloword.cpp : 기본 main 코드
        - helloOpenCV.cpp : OpenCV 사용 코드(외부 라이브러리 적용)
        - hellow_publisher : Node 및 spin timer 적용
        - hellow_publisher_class : Node 상속 클래스 적용
---------------------------------------------------
 2024-07-31

오전 수업
- 복습
- 패키지 작성 (이어서)
  - kdta_ws
    - simple_pkg_cpp 패키지
      - 노드 추가
        - hellow_publisher_class2 : 분할컴파일 (해더, 메인, CPP)
        - hellow_subscriber : subscriber 노드
        - time_publisher_class : timer 노드
        - time_subscriber : timer subscriber 노드
    - move_t_sim 패키지
      - 노드 추가
        - moveTurtleSim.cpp : 터틀심 이동 노드( turtle1, turtle2 동시 이동)

오후 수업
- 최종 실습(Topic)
  - 5개 노드 3개 Topic 사용
    - 노드 : mpub, msub, msub2, tpub, mtsub
    - Topic :
      - /message1: String ( mpub -> msub, mtsub)
      - /message2: String ( mpub -> msub2)
      - /time: Header ( tpub -> mtsub)
---------------------------------------------------
2024-08-01

- 복습
- 터틀봇3 네트워크 설정
  - 공유기 설치
    - ssid: turtlebot3
    - password: turtlebot3
  - 가상환경 bridge 설정
    - 네트워크 브릿지 클리 후 네트워크 카드 하나만 사용
- 터틀봇3 개요
  - 터틀봇3 소개
  - 터틀봇3 구성
  - 터틀봇3 시스템 구성
- 터틀봇3 ssh 연결
  - 터미널에서 연결
    - `ssh ubuntu@192.169.14.XX`
      - userid: ubuntu
      - password: aa
    - vscode remote ssh 연결
      - vscode remote ssh 설정
- 터틀봇3 브링업
  - 터틀봇3 브링업
    - ROS_DOMAIN_ID 확인
    - `export TURTLEBOT3_MODEL=burger`
    - 브링업 런치 실행 `ros2 launch turtlebot3_bringup robot.launch.py`
- 텔레옵키 노드 실행(노트북)
  - 텔레옵키 노드 실행
    - `ros2 run turtlebot3_teleop teleop_keyboard`
- gazebo
  - gazebo 설치
    - `sudo apt install ros-humble-gazebo-*`
  - turtlebot3 simulation 설치(깃 클론)
    - `cd ~/kdta_ws/src`
    - `git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`
    - `cd ~/turtlebot3_ws && colcon build --symlink-install`
  - turtlebot3 simulation 실행
    - `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
- 터틀봇3에서 빌드 후 실행
  - move_t_sim 이동
    - vscode 로컬과 원격을 동시 실행 후 이동
    - 빌드 후 실행(터틀봇3)
      - `cd ~/robot_ws`
      - `colcon build --symlink-install`
      - `ros2 run move_t_sim moveTurtleBot`
- 터틀심 사각형 움직임 구현
  - moveTurtleSim2.cpp 작성
  - 빌드 후 실행(터틀봇3)
    - `ros2 run move_t_sim moveTurtleSim2`
---------------------------------------------------
2024-08-02

- 복습
- 터틀심 사각형 움직임 구현 -> 터틀봇 용 코드로 변경
  - moveTurtleBot2.cpp 작성
  - 세부 알고리즘 변경
- 사용자 인터페이스 작성
  - srv/AddTwoInt.srv 파일 작성
- Service 노드 추가
  - Service server 노드
  - Service client 노드1
    - spin_until_done 사용
  - Service client 노드2
    - spin 사용 후 callback 사용
- 파라미터
  - 파라미터 기초
    - ros2 param get /node_name param_name
    - ros2 param set /node_name param_name param_value
    - ros2 param list /node_name
    - ros2 param dump /node_name
    - ros2 param load /node_name
- Launch 파일 작성
  - topic_final 에 추가

