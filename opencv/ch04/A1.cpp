/* 실습 문제 1번
1. 레나 이미지 WINDOW
2. 마우스를 따라다니는 사각형 [50,50]
3. 마우스 클릭시 마우스위치의 RGB값을 화면에 출력
4. RGB 값을 VECTOR 에 넣고 FILESTORAGE 에 저장
   "mouse_RGBvector.json"
5. (시간되면) 최적화 () 까지
*/

#include "opencv2/freetype.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/ha/Academy_ROS2/opencv/data/";
